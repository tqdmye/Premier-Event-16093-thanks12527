package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ColorDetectionExample extends LinearOpMode {
    OpenCvWebcam webcam;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final double targetX = 90;
    private final double targetY = 60;
    private double Kp = 0.01;
    private double Ki = 0.00;
    private double Kd = 0.01;
    private double previousErrorX = 0;
    private double previousErrorY = 0;
    private double integralX = 0;
    private double integralY = 0;
    private ColorPipeline pipeline;

    @Override
    public void runOpMode() {
        // 获取摄像头ID
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        // 初始化摄像头
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        // 初始化 ColorPipeline
        pipeline = new ColorPipeline();  // 显式创建 pipeline 对象
        webcam.setPipeline(pipeline);  // 将 pipeline 设置给摄像头

        // 初始化电机
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        webcam.setMillisecondsPermissionTimeout(5000);

        // 开启摄像头，确保异步开启后再开始流传输
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // 等待1000ms再开始流传输
                sleep(1000);
                webcam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open: " + errorCode);
                telemetry.update();
            }
        });

        // 等待启动
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 获取识别到的颜色
            String color = pipeline.getDetectedColor();
            double x = pipeline.getCenterX();
            double y = pipeline.getCenterY();

            telemetry.addData("Color", color);
            telemetry.addData("Center X", x);
            telemetry.addData("Center Y", y);
            telemetry.update();

            double xOffset = targetX - x;
            double yOffset = targetY - y;
            double threshold = 5;

            if (Math.abs(xOffset) > threshold || Math.abs(yOffset) > threshold) {
                double xPower = calculatePID(xOffset, 'x');
                double yPower = calculatePID(yOffset, 'y');
                double num = 0.2;
                frontLeft.setPower((yPower + xPower) * num);
                frontRight.setPower((yPower - xPower) * num);
                backLeft.setPower((yPower - xPower) * num);
                backRight.setPower((yPower + xPower) * num);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
        }
    }

    // PID计算
    private double calculatePID(double offset, char axis) {
        double error = offset;
        double integral, derivative;

        if (axis == 'x') {
            integralX += error;
            derivative = error - previousErrorX;
            previousErrorX = error;
            integral = integralX;
        } else {
            integralY += error;
            derivative = error - previousErrorY;
            previousErrorY = error;
            integral = integralY;
        }

        return Kp * error + Ki * integral + Kd * derivative;
    }

    class ColorPipeline extends OpenCvPipeline {
        private Mat hsv = new Mat();
        private String detectedColor = "None";
        private double centerX = 0;
        private double centerY = 0;

        private Scalar lowerRed1 = new Scalar(0, 70, 50);
        private Scalar upperRed1 = new Scalar(10, 255, 255);
        private Scalar lowerRed2 = new Scalar(160, 70, 50);
        private Scalar upperRed2 = new Scalar(180, 255, 255);
        private Scalar lowerBlue = new Scalar(90, 70, 50);
        private Scalar upperBlue = new Scalar(130, 255, 255);
        private Scalar lowerYellow = new Scalar(20, 70, 50);
        private Scalar upperYellow = new Scalar(40, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat redMask1 = new Mat(), redMask2 = new Mat(), redMask = new Mat();
            Mat blueMask = new Mat(), yellowMask = new Mat(), totalMask = new Mat();

            Core.inRange(hsv, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsv, lowerRed2, upperRed2, redMask2);
            Core.bitwise_or(redMask1, redMask2, redMask);
            Core.inRange(hsv, lowerBlue, upperBlue, blueMask);
            Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

            Core.bitwise_or(redMask, blueMask, totalMask);
            Core.bitwise_or(totalMask, yellowMask, totalMask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(totalMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
                Point[] vertices = new Point[4];
                rotatedRect.points(vertices);

                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                }

                centerX = rotatedRect.center.x;
                centerY = rotatedRect.center.y;

                Mat roi = hsv.submat(rotatedRect.boundingRect());
                detectedColor = determineColor(roi);
                roi.release();

                Scalar boxColor;
                switch (detectedColor) {
                    case "Red": boxColor = new Scalar(255, 0, 0); break;
                    case "Blue": boxColor = new Scalar(0, 0, 255); break;
                    case "Yellow": boxColor = new Scalar(0, 255, 255); break;
                    default: boxColor = new Scalar(255, 255, 255); break;
                }

                Imgproc.circle(input, new Point(centerX, centerY), 5, boxColor, -1);
                Imgproc.putText(input, detectedColor, new Point(rotatedRect.boundingRect().x, rotatedRect.boundingRect().y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, boxColor, 2);
            } else {
                detectedColor = "None";
                centerX = 0;
                centerY = 0;
            }

            redMask1.release();
            redMask2.release();
            redMask.release();
            blueMask.release();
            yellowMask.release();
            totalMask.release();

            return input;
        }

        private String determineColor(Mat roi) {
            Mat maskR = new Mat(), maskB = new Mat(), maskY = new Mat();

            Core.inRange(roi, lowerRed1, upperRed1, maskR);
            Core.inRange(roi, lowerBlue, upperBlue, maskB);
            Core.inRange(roi, lowerYellow, upperYellow, maskY);

            double redSum = Core.sumElems(maskR).val[0];
            double blueSum = Core.sumElems(maskB).val[0];
            double yellowSum = Core.sumElems(maskY).val[0];

            maskR.release();
            maskB.release();
            maskY.release();

            if (redSum > blueSum && redSum > yellowSum) return "Red";
            if (blueSum > redSum && blueSum > yellowSum) return "Blue";
            if (yellowSum > redSum && yellowSum > blueSum) return "Yellow";
            return "Unknown";
        }

        public String getDetectedColor() {
            return detectedColor;
        }

        public double getCenterX() {
            return centerX;
        }

        public double getCenterY() {
            return centerY;
        }
    }
}
