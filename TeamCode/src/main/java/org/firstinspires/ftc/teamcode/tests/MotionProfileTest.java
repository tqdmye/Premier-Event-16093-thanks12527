package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Motion profile")
public class MotionProfileTest extends LinearOpMode {
  private final Telemetry telemetry_M =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  public static double set_position = 0;
  public static double last_set_position = 0;

  public static boolean read_only = false;
  public static boolean reverse_1 = false;
  public static boolean reverse_2 = false;
  public static boolean brake = false;

  public static double maxVelocity = 12000;
  public static double maxAcceleration = 12000;

  public static String motor_name_master = "liftMotorUp";
  public static String motor_name_slave = "liftMotorDown";
  public static double Kp = 0.01, Ki = 0.0, Kd = 0.0;
  public static double kS = 0.0, kV = 0.0, kG = 0.12;
  public final PIDController motorPID = new PIDController(Kp, Ki, Kd);

  private final ElapsedTime timer = new ElapsedTime();

  private ElevatorFeedforward feedforward;
  private TrapezoidProfile profile;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double lastTime;

  private Motor motor1;
  private Motor motor2;

  @Override
  public void runOpMode() throws InterruptedException {
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    feedforward = new ElevatorFeedforward(kS, kG, kV);

    motor1 = new Motor(hardwareMap, motor_name_master);
    motor2 = new Motor(hardwareMap, motor_name_slave);
    motor1.stopAndResetEncoder();
    motor2.stopAndResetEncoder();
    motor1.setRunMode(Motor.RunMode.RawPower);
    motor2.setRunMode(Motor.RunMode.RawPower);

    timer.reset();

    waitForStart();

    if (reverse_1) {
      motor1.setInverted(true);
    }

    if (reverse_2) {
      motor2.setInverted(true);
    }

    if (brake) {
      motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    timer.startTime();
    lastTime = timer.time(TimeUnit.MILLISECONDS);

    while (opModeIsActive()) {
      if (last_set_position != set_position) {
        last_set_position = set_position;
        goal = new TrapezoidProfile.State(set_position, 0);
      }

      setpoint =
          profile.calculate(
              Range.clip((timer.time(TimeUnit.MILLISECONDS) - lastTime) * 0.001, 0.001, 0.02),
              setpoint,
              goal);

      lastTime = timer.time(TimeUnit.MILLISECONDS);

      if (!read_only) {
        double pidPower = motorPID.calculate(motor1.getCurrentPosition(), setpoint.position);
        double output = pidPower + feedforward.calculate(setpoint.velocity);
        motor1.set(Range.clip(output, -1, 1));
        motor2.set(Range.clip(output, -1, 1));

        telemetry_M.addData("PID Power", pidPower);
        telemetry_M.addData("Actual power", output);
      }

      telemetry_M.addData("encoder_1", motor1.getCurrentPosition());
      telemetry_M.addData("encoder_2", motor1.getCurrentPosition());

      telemetry_M.addData("motor_up_position_ticks", motor1.getCurrentPosition());

      telemetry_M.addData("velocity_up_ticksPerSecond", motor1.encoder.getRawVelocity());
      telemetry_M.addData(
          "acceleration_up_ticksPerSecondSquared", motor1.encoder.getAcceleration());

      telemetry_M.addData(
          "Time Interval",
          Range.clip((timer.time(TimeUnit.MILLISECONDS) - lastTime) * 0.001, 0.001, 0.02));

      telemetry_M.update();
    }
  }
}
