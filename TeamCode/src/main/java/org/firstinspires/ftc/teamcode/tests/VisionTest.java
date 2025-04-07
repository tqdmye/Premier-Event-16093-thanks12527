package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@TeleOp(name = "LimelightTest")
public class VisionTest extends LinearOpMode {
  public static Vision.SampleColor color = Vision.SampleColor.RED;
  private Limelight3A camera;
  public static double colorChoice = 0.0;

  // First Index 0.0 Red 1.0 Blue 2.0 Yellow
  @Override
  public void runOpMode() throws InterruptedException {
    // camera = hardwareMap.get(Limelight3A.class, "limelight");
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Vision vision = new Vision(hardwareMap, telemetry);
    // camera.resetDeviceConfigurationForOpMode();
    //    camera.setPollRateHz(100);
    //    camera.start();
    vision.initializeCamera();
    waitForStart();

    while (!isStopRequested() && opModeIsActive()) {
      CommandScheduler.getInstance().run();

      vision.setDetectionColor(color);
      telemetry.addData("Distance", vision.getDistance());
      telemetry.addData("TX", vision.getTx(0.0));
      telemetry.addData("Strafe", vision.getStrafeOffset());
      telemetry.addData("TY", vision.getTy(0.0));
      telemetry.addData("Old Data", vision.isDataOld());
      telemetry.addData("Area", vision.isTargetVisible());
      telemetry.update();
      //      boolean isUploaded =
      //          camera.updatePythonInputs(new double[] {colorChoice, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      // 0.0});
      //
      //      LLResult result = camera.getLatestResult();
      //
      //      telemetry.addData("Result Null", result == null ? "Yes" : "No");
      //      telemetry.addData("Result Valid", result.isValid() ? "Yes" : "No");
      //
      //      if (result != null) {
      //        long staleness = result.getStaleness();
      //        if (staleness < 100) { // Less than 100 milliseconds old
      //          telemetry.addData("Data", "Good");
      //        } else {
      //          telemetry.addData("Data", "Old (" + staleness + " ms)");
      //        }
      //
      //        double tx = result.getTx();
      //        double ty = result.getTy();
      //        double ta = result.getTa();
      //        telemetry.addData("TX: Degrees", tx);
      //        telemetry.addData("TY: Degrees", ty);
      //        telemetry.addData("TA: Area", ta);
      //      }
      //
      //      telemetry.addData("Is Uploaded", isUploaded);
      //      telemetry.update();
    }

    CommandScheduler.getInstance().reset();
  }
}
