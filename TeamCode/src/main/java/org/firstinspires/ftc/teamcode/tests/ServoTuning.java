package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Config
@TeleOp(name = "Easy Servo Tuning")
public class ServoTuning extends LinearOpMode {
  public static double originalPos = 0;
  public static double originalSmallerPos = -1;
  public static double originalBiggerPos = -1;
  public static double currentSmallerPos = -1;
  public static double currentBiggerPos = -1;

  public static double servo_pos1 = 0.5;
  private Servo servo0 = null;
  public static String servo_name1 = "servo3";
  private final Telemetry mTelemetry =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

  @Override
  public void runOpMode() throws InterruptedException {
    servo0 = hardwareMap.get(Servo.class, servo_name1);
    waitForStart();
    while (opModeIsActive()) {
      servo0.setPosition(servo_pos1);
      if (originalSmallerPos != -1
          && originalBiggerPos != -1
          && currentSmallerPos != -1
          && currentBiggerPos != -1) {
        mTelemetry.addData("Original Pose", originalPos);
        mTelemetry.addData(
            "Transformed Pose",
            MathUtils.linear(
                originalPos,
                originalSmallerPos,
                originalBiggerPos,
                currentSmallerPos,
                currentBiggerPos));
      }
      mTelemetry.addData("Current Position", servo_pos1);
      mTelemetry.update();
    }
  }
}
