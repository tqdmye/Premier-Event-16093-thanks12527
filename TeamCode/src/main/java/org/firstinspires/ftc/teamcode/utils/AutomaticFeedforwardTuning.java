package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
@TeleOp(name = "Automatic FF Tuning")
public class AutomaticFeedforwardTuning extends LinearOpMode {
  public static boolean isFinished = false;
  public static double FF_RAMP_RATE = 0.1;
  private SampleMecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {
    boolean isStart = false;
    Telemetry mTelemetry =
        new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    drive = new SampleMecanumDrive(hardwareMap);
    velocitySamples.clear();
    voltageSamples.clear();

    waitForStart();

    while (opModeIsActive()) {
      if (!isStart) {
        timer.reset();
        isStart = true;
      }

      if (isFinished) {
        drive.runCharacterization(0);
        int n = velocitySamples.size();
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;
        for (int i = 0; i < n; i++) {
          sumX += velocitySamples.get(i);
          sumY += voltageSamples.get(i);
          sumXY += velocitySamples.get(i) * voltageSamples.get(i);
          sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
        }
        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        NumberFormat formatter = new DecimalFormat("#0.00000");
        mTelemetry.addData("kS", formatter.format(kS));
        mTelemetry.addData("kV", formatter.format(kV));
        mTelemetry.update();
      } else {
        double voltage = timer.time() * FF_RAMP_RATE;
        drive.runCharacterization(voltage);
        velocitySamples.add(drive.getXVelocity());
        voltageSamples.add(voltage);
      }
    }
  }
}
