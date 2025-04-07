package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.headingPoseError;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.xPoseError;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.yPoseError;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.SQPIDHolonomicFollower;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive implements Subsystem {
  public static PIDCoefficients FAST_TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0.5);
  public static PIDCoefficients FAST_HEADING_PID = new PIDCoefficients(2, 0, 0.15);

  public static PIDCoefficients MED_TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0.3);
  public static PIDCoefficients MED_HEADING_PID = new PIDCoefficients(2, 0, 0);

  public static PIDCoefficients SLOW_TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0.05);
  public static PIDCoefficients SLOW_HEADING_PID = new PIDCoefficients(2.3, 0, 0);

  @Setter public TrajectoryMode currentTrajectoryMode = TrajectoryMode.FAST;

  public static double LATERAL_MULTIPLIER = 1.4514;

  public static double VX_WEIGHT = 1;
  public static double VY_WEIGHT = 1;
  public static double OMEGA_WEIGHT = 1;

  public static double SLOW_ADMISSIBLE_TIMEOUT = 0.3;

  public static double MED_ADMISSIBLE_TIMEOUT = 0.2;

  private TrajectorySequenceRunner fastTrajectorySequenceRunner;
  private TrajectorySequenceRunner medTrajectorySequenceRunner;
  private TrajectorySequenceRunner slowTrajectorySequenceRunner;

  private static final TrajectoryVelocityConstraint VEL_CONSTRAINT =
      getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
  private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT =
      getAccelerationConstraint(MAX_ACCEL);

  private TrajectoryFollower fastFollower;
  private TrajectoryFollower medFollower;
  private TrajectoryFollower slowFollower;

  private DcMotorEx leftFront, leftRear, rightRear, rightFront;
  private List<DcMotorEx> motors;

//  private GoBildaLocalizer od;
  private VoltageSensor batteryVoltageSensor;

  private List<Integer> lastEncPositions = new ArrayList<>();
  private List<Integer> lastEncVels = new ArrayList<>();

  public static double yawHeading = 0;

  private Telemetry telemetry;
  IMUEncoderLocalizer imu;

  public SampleMecanumDrive(HardwareMap hardwareMap) {
    super(
        kV,
        kA,
        kStatic,
        TRACK_WIDTH,
        TRACK_WIDTH,
        LATERAL_MULTIPLIER); // Drive Constants are passed to here

    fastFollower =
        new HolonomicPIDVAFollower(
            FAST_TRANSLATIONAL_PID,
            FAST_TRANSLATIONAL_PID,
            FAST_HEADING_PID,
            new Pose2d(1.5, 1.5, Math.toRadians(2)), // Pose Error
            SLOW_ADMISSIBLE_TIMEOUT);

    medFollower =
        new SQPIDHolonomicFollower(
            SLOW_TRANSLATIONAL_PID,
            SLOW_TRANSLATIONAL_PID,
            SLOW_HEADING_PID,
            new Pose2d(0.5, 0.5, Math.toRadians(2)), // Pose Error
            MED_ADMISSIBLE_TIMEOUT);

    slowFollower =
        new SQPIDHolonomicFollower(
            SLOW_TRANSLATIONAL_PID,
            SLOW_TRANSLATIONAL_PID,
            SLOW_HEADING_PID,
            new Pose2d(xPoseError, yPoseError, Math.toRadians(headingPoseError)), // Pose Error
            SLOW_ADMISSIBLE_TIMEOUT);

    // LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // TODO: adjust the names of the following hardware devices to match your configuration
//    od = new GoBildaLocalizer(hardwareMap, DriveConstants.GoBildaLocalizerPerpendicularOffset);
    imu = new IMUEncoderLocalizer(hardwareMap);
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
    leftRear = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
    rightRear = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

    motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

    for (DcMotorEx motor : motors) {
      MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
      motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
      motor.setMotorType(motorConfigurationType);
    }

    if (RUN_USING_ENCODER) {
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
      setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
    }

    // TODO: reverse any motors using DcMotor.setDirection()

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();

    // TODO: if desired, use setLocalizer() to change the localization method
//    setLocalizer(od);

    fastTrajectorySequenceRunner =
        new TrajectorySequenceRunner(
            fastFollower,
            FAST_HEADING_PID,
            batteryVoltageSensor,
            lastEncPositions,
            lastEncVels,
            lastTrackingEncPositions,
            lastTrackingEncVels);

    medTrajectorySequenceRunner =
        new TrajectorySequenceRunner(
            medFollower,
            MED_HEADING_PID,
            batteryVoltageSensor,
            lastEncPositions,
            lastEncVels,
            lastTrackingEncPositions,
            lastTrackingEncVels);

    slowTrajectorySequenceRunner =
        new TrajectorySequenceRunner(
            slowFollower,
            SLOW_HEADING_PID,
            batteryVoltageSensor,
            lastEncPositions,
            lastEncVels,
            lastTrackingEncPositions,
            lastTrackingEncVels);

    CommandScheduler.getInstance().registerSubsystem(this);
    this.telemetry = FtcDashboard.getInstance().getTelemetry();
  }

  public SampleMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
    this(hardwareMap);
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  }



//  public void resetPinpoint() {
//    od.recalibrateIMU();
//  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
    return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
    return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
    return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
    return new TrajectorySequenceBuilder(
        startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL);
  }

  public void turnAsync(double angle) {
    switch (currentTrajectoryMode) {
      case FAST:
        fastTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
        break;
      case MEDIUM:
        medTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
        break;
      case SLOW:
        slowTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
        break;
    }
  }

  public void runCharacterization(double output) {
    setWeightedDrivePower(new Pose2d(output, 0, 0));
  }

//  public double getXVelocity() {
//    return od.getPoseVelocity().getX();
//  }

  public void turn(double angle) {
    turnAsync(angle);
    waitForIdle();
  }

  public void followTrajectoryAsync(Trajectory trajectory) {
    switch (currentTrajectoryMode) {
      case FAST:
        fastTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
        break;
      case MEDIUM:
        medTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
        break;
      case SLOW:
        slowTrajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
        break;
    }
  }

  public void followTrajectory(Trajectory trajectory) {
    followTrajectoryAsync(trajectory);
    waitForIdle();
  }

  public void followTrajectorySequenceLineLinearAsync(Pose2d goalPose) {
    followTrajectorySequenceAsync(
        TrajectoryManager.trajectorySequenceBuilder(getPoseEstimate())
            .lineToLinearHeading(goalPose)
            .build());
  }

  public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
    switch (currentTrajectoryMode) {
      case FAST:
        fastTrajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
        break;
      case MEDIUM:
        medTrajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
        break;
      case SLOW:
        slowTrajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
        break;
    }
  }

  public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
    followTrajectorySequenceAsync(trajectorySequence);
    waitForIdle();
  }

  public Pose2d getLastError() {
    switch (currentTrajectoryMode) {
      case FAST:
        return fastTrajectorySequenceRunner.getLastPoseError();
      case MEDIUM:
        return medTrajectorySequenceRunner.getLastPoseError();
      case SLOW:
        return slowTrajectorySequenceRunner.getLastPoseError();
    }
    return new Pose2d();
  }

  public void update() {
    updatePoseEstimate();
    DriveSignal signal = null;
    switch (currentTrajectoryMode) {
      case FAST:
        signal = fastTrajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        break;
      case MEDIUM:
        signal = medTrajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        break;
      case SLOW:
        signal = slowTrajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        break;
    }
    if (signal != null) setDriveSignal(signal);
  }

  public void waitForIdle() {
    while (!Thread.currentThread().isInterrupted() && isBusy()) update();
  }

  public boolean isBusy() {
    return fastTrajectorySequenceRunner.isBusy()
        || medTrajectorySequenceRunner.isBusy()
        || slowTrajectorySequenceRunner.isBusy();
  }

  public void breakFollowing(boolean cancelAll) {
    if (cancelAll) {
      fastTrajectorySequenceRunner.breakFollowing();
      medTrajectorySequenceRunner.breakFollowing();
      slowTrajectorySequenceRunner.breakFollowing();
      return;
    }

    switch (currentTrajectoryMode) {
      case FAST:
        fastTrajectorySequenceRunner.breakFollowing();
        break;
      case MEDIUM:
        medTrajectorySequenceRunner.breakFollowing();
        break;
      case SLOW:
        slowTrajectorySequenceRunner.breakFollowing();
        break;
    }
  }

  public void setMode(DcMotor.RunMode runMode) {
    for (DcMotorEx motor : motors) {
      motor.setMode(runMode);
    }
  }

  public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
    for (DcMotorEx motor : motors) {
      motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
  }

  public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
    PIDFCoefficients compensatedCoefficients =
        new PIDFCoefficients(
            coefficients.p,
            coefficients.i,
            coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.getVoltage());

    for (DcMotorEx motor : motors) {
      motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
  }

  public void setFieldRelativeDrivePower(Pose2d drivePower) {
    Pose2d vel = drivePower;
//    double botHeading = od.getHeading() - yawHeading;
    double botHeading = imu.getPoseEstimate().getHeading() - yawHeading;

    // Rotate the movement direction counter to the bot's rotation
    double rotX = vel.getX() * Math.cos(-botHeading) - vel.getY() * Math.sin(-botHeading);
    double rotY = vel.getX() * Math.sin(-botHeading) + vel.getY() * Math.cos(-botHeading);

    double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(drivePower.getHeading()), 1);
    vel = new Pose2d(rotX, rotY, drivePower.getHeading()).div(denom);

    setDrivePower(vel);
  }

  public double getHeading() {
    return imu.getPoseEstimate().getHeading() - yawHeading;
  }

  public void resetHeading() {
    yawHeading = imu.getPoseEstimate().getHeading();
  }

  public void setWeightedDrivePower(Pose2d drivePower) {
    Pose2d vel = drivePower;

    if (Math.abs(drivePower.getX())
            + Math.abs(drivePower.getY())
            + Math.abs(drivePower.getHeading())
        > 1) {
      // re-normalize the powers according to the weights
      double denom =
          VX_WEIGHT * Math.abs(drivePower.getX())
              + VY_WEIGHT * Math.abs(drivePower.getY())
              + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

      vel =
          new Pose2d(
                  VX_WEIGHT * drivePower.getX(),
                  VY_WEIGHT * drivePower.getY(),
                  OMEGA_WEIGHT * drivePower.getHeading())
              .div(denom);
    }

    setDrivePower(vel);
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() { // Is it correct?
    lastEncPositions.clear();

    List<Double> wheelPositions = new ArrayList<>();
    for (DcMotorEx motor : motors) {
      int position = motor.getCurrentPosition();
      lastEncPositions.add(position);
      wheelPositions.add(encoderTicksToInches(position));
    }
    return wheelPositions;
  }

  @Override
  public List<Double> getWheelVelocities() {
    lastEncVels.clear();

    List<Double> wheelVelocities = new ArrayList<>();
    for (DcMotorEx motor : motors) {
      int vel = (int) motor.getVelocity();
      lastEncVels.add(vel);
      wheelVelocities.add(encoderTicksToInches(vel));
    }
    return wheelVelocities;
  }

  @Override
  public void setMotorPowers(double v, double v1, double v2, double v3) {
    leftFront.setPower(v);
    leftRear.setPower(v1);
    rightRear.setPower(v2);
    rightFront.setPower(v3);
  }

  @Override
  public double getRawExternalHeading() {
    return imu.getPoseEstimate().getHeading();
  }

  @Override
  public Double getExternalHeadingVelocity() {
    return imu.getHeadingVelocity();
  }

  public double getDeltaX(Pose2d initalPose) {
    Pose2d currentPose = imu.getPoseEstimate();
    return (currentPose.getX() - initalPose.getX()) * Math.cos(currentPose.getHeading())
        + (currentPose.getY() - initalPose.getY()) * Math.sin(currentPose.getHeading());
  }

  public double getDeltaY(Pose2d initalPose) {
    Pose2d currentPose = imu.getPoseEstimate();
    return (currentPose.getX() - initalPose.getX()) * Math.sin(currentPose.getHeading())
        + (currentPose.getY() - initalPose.getY()) * Math.cos(currentPose.getHeading());
  }

  public static TrajectoryVelocityConstraint getVelocityConstraint(
      double maxVel, double maxAngularVel, double trackWidth) {
    return new MinVelocityConstraint(
        Arrays.asList(
            new AngularVelocityConstraint(maxAngularVel),
            new MecanumVelocityConstraint(maxVel, trackWidth)));
  }

  public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
    return new ProfileAccelerationConstraint(maxAccel);
  }

  public enum TrajectoryMode {
    SLOW,
    MEDIUM,
    FAST
  }

  @Override
  public void periodic() {
    imu.update();
  }
}
