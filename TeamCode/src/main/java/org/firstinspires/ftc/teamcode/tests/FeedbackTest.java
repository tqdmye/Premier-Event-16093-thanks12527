package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.FeedBackServo;

@Config
@TeleOp(name = "FeedbackTest")
public class FeedbackTest extends LinearOpMode {
  private final Telemetry telemetry_M =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  private FeedBackServo feedBackServo;
  public static double servo_pos = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    feedBackServo = new FeedBackServo(hardwareMap, "intakeClawServo", "servoPos");
    waitForStart();
    while (opModeIsActive()) {
      feedBackServo.setPosition(servo_pos);
      telemetry_M.addData("Setpoint", feedBackServo.getServoSetpoint());
      telemetry_M.addData("Actual Position", feedBackServo.getServoPosition());
      telemetry_M.update();
    }
  }
}
