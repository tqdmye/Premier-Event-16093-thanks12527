package org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

// @TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

  public static void drawRobot(Canvas c, Pose2d t) {
    final double ROBOT_RADIUS = 9;

    c.setStrokeWidth(1);
    c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

    Vector2d halfv = new Vector2d(1, 0).rotated(t.getHeading()).times(0.5 * ROBOT_RADIUS);
    Vector2d p1 = new Vector2d(t.getX(), t.getY()).plus(halfv);
    Vector2d p2 = p1.plus(halfv);
    c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
  }

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    drive.setPoseEstimate(new Pose2d(54, -54));

    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();

    while (opModeIsActive()) {
      drive.setFieldRelativeDrivePower(
          new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
      if (gamepad1.dpad_left) {
        drive.resetHeading();
      }

      drive.update();

      Pose2d pose = drive.getPoseEstimate();
      telemetry.addData("x", pose.getX());
      telemetry.addData("y", pose.getY());
      telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
      telemetry.addData("xVelocity:", drive.getPoseVelocity().getX());
      telemetry.addData("yVelocity:", drive.getPoseVelocity().getY());
      telemetry.update();

      TelemetryPacket packet = new TelemetryPacket();
      packet.fieldOverlay().setStroke("#3F51B5");
      drawRobot(packet.fieldOverlay(), pose);
      FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
  }
}
