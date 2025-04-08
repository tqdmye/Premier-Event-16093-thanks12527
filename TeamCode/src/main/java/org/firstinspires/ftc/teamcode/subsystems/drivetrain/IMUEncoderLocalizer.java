package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//现在使用odometry computer，这个废止

public class IMUEncoderLocalizer implements Localizer {
    private final DcMotorEx xEncoder;
    private final DcMotorEx yEncoder;
    private final IMU imu;

    private Pose2d currentPose = new Pose2d(0, 0, 0);
    private double lastXPos, lastYPos;

    public IMUEncoderLocalizer(HardwareMap hardwareMap) {
//        xEncoder = hardwareMap.get(DcMotorEx.class, "xEncoder");
//        yEncoder = hardwareMap.get(DcMotorEx.class, "yEncoder");
        xEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        yEncoder = hardwareMap.get(DcMotorEx.class,"leftBack");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,    //should have a check
                        RevHubOrientationOnRobot.UsbFacingDirection.UP  //should have a check
                )
        );
        imu.initialize(parameters);
        resetEncoders();
        imu.resetYaw();
    }

    private void resetEncoders() {
        xEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastXPos = 0;
        lastYPos = 0;
    }

    private double encoderTicksToMeters(double ticks) {
        return ticks * DriveConstants.EnCODER_RESOLUTION_INCH_PER_TICK;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        currentPose = pose;
        resetEncoders();
        imu.resetYaw();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        double xVel = encoderTicksToMeters(xEncoder.getVelocity());
        double yVel = encoderTicksToMeters(yEncoder.getVelocity());
        return new Pose2d(xVel, yVel, 0);
    }

    @Override
    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.RADIANS);

        double currentXPos = encoderTicksToMeters(xEncoder.getCurrentPosition());
        double currentYPos = encoderTicksToMeters(yEncoder.getCurrentPosition());
        double deltaX = currentXPos - lastXPos;
        double deltaY = currentYPos - lastYPos;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double globalDeltaX = deltaX * cos - deltaY * sin;
        double globalDeltaY = deltaX * sin + deltaY * cos;

        currentPose = new Pose2d(
                currentPose.getX() + globalDeltaX,
                currentPose.getY() + globalDeltaY,
                heading
        );

        lastXPos = currentXPos;
        lastYPos = currentYPos;
    }

    public double getHeadingVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }
}
