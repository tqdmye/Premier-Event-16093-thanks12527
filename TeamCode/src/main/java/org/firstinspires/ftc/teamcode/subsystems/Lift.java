package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.concurrent.TimeUnit;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Config
public class Lift extends MotorPIDSlideSubsystem {
  public static double kP = 0.007, kI = 0.0, kD = 0.0, kV = 0, kS = 0, kG = 0.04;
  public static double autoBasketHeight = 750;
  private final PIDController pidController;
  private final Motor liftMotorUp;
  private final Motor liftMotorDown;

  private double lastSetpoint = 0;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private final ElapsedTime timer;
  private double lastTime;

  //  private boolean isResetting = false;
  public static double resetPower = -0.7;
  public static double hangAdditionalPower = 0;

  public static double MAX_VEL = 0;
  public static double MAX_ACL = 0;

  public static double PRE_HANG_TICKS = 250;

  private final ElevatorFeedforward feedforward;

  private final VoltageSensor batteryVoltageSensor;

  @Getter @Setter private Goal goal = Goal.STOW;

  public Lift(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftMotorUp = new Motor(hardwareMap, "liftUp");
    liftMotorDown = new Motor(hardwareMap, "liftDown");
    liftMotorUp.setInverted(true);
    liftMotorDown.setInverted(true);
    liftMotorUp.stopAndResetEncoder();
    liftMotorDown.stopAndResetEncoder();
    liftMotorUp.setRunMode(Motor.RunMode.RawPower);
    liftMotorDown.setRunMode(Motor.RunMode.RawPower);

    pidController = new PIDController(kP, kI, kD);
    pidController.setIntegrationBounds(-1 / kI, 1 / kI);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    this.telemetry = telemetry;

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACL));
    timer = new ElapsedTime();
    timer.reset();
    lastTime = timer.time(TimeUnit.MILLISECONDS);
  }

  public void runOpenLoop(double percent) {
//        goal = Goal.OPEN_LOOP;
    // TODO: move this to MotorPIDSlideSubsystem
    if (percent == 0) {
      isResetting = false;
    } else {
      isResetting = true;
    }
    double output = Range.clip(percent, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);
  }

  public double getResetPower() {
    return resetPower;
  }

  public void resetEncoder() {
    runOpenLoop(0);
    pidController.reset();
    pidController.calculate(0);
    goal = Goal.STOW;
    // TODO: does this work?
    liftMotorUp.resetEncoder();
    liftMotorDown.resetEncoder();
    telemetry.addData("Lift.Current Position", getCurrentPosition());
    telemetry.addData("Lift.Error", pidController.getPositionError());
    // telemetry.update();
  }

  public long getCurrentPosition() {
    return liftMotorUp.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, getCurrentPosition(), 8);
  }

  public boolean atHome(double tolerance) {
    return MathUtils.isNear(Goal.STOW.setpointTicks, getCurrentPosition(), tolerance);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, getCurrentPosition(), 10);
  }

  public void periodicAsync() {
    if (goal == Goal.OPEN_LOOP || isResetting) return;

    if (lastSetpoint != goal.setpointTicks) {
      goalState = new TrapezoidProfile.State(goal.setpointTicks, 0);
      lastSetpoint = goal.setpointTicks;
    }

    double timeInterval =
        Range.clip((timer.time(TimeUnit.MILLISECONDS) - lastTime) * 0.001, 0.001, 0.05);

    setpointState = profile.calculate(timeInterval, setpointState, goalState);

    double pidPower = pidController.calculate(getCurrentPosition(), setpointState.position);
    double output = pidPower + feedforward.calculate(setpointState.velocity);
    if (goal == Goal.HANG) {
      output += hangAdditionalPower;
    }
    //    output *= 12 / batteryVoltageSensor.getVoltage();
    output = Range.clip(output, -1, 1);
    liftMotorUp.set(output);
    liftMotorDown.set(output);

    lastTime = timer.time(TimeUnit.MILLISECONDS);

    // telemetry.update();
  }

  public boolean isPreHang() {
    return liftMotorUp.getCurrentPosition() > 200;
  }

  public enum Goal {
    HIGH_BASKET(760.0),
    AUTO_BASKET(autoBasketHeight),
    LOW_BASKET(200),
    AUTO_TRANSFER(100),
    STOW(0.0),
    PRE_HANG(PRE_HANG_TICKS),
    HANG(0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }
  }
}
