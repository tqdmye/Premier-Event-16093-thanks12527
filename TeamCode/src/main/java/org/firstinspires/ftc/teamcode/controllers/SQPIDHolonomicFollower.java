package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;

public class SQPIDHolonomicFollower extends TrajectoryFollower {
  private final SQPIDController axialController;
  private final SQPIDController lateralController;
  private final SQPIDController headingController;
  private Pose2d lastError;

  public SQPIDHolonomicFollower(
      PIDCoefficients axialCoeffs,
      PIDCoefficients lateralCoeffs,
      PIDCoefficients headingCoeffs,
      Pose2d admissibleError,
      double timeout) {
    super(admissibleError, timeout, NanoClock.system());

    this.axialController = new SQPIDController(axialCoeffs.kP, axialCoeffs.kI, axialCoeffs.kD);

    this.lateralController =
        new SQPIDController(lateralCoeffs.kP, lateralCoeffs.kI, lateralCoeffs.kD);

    this.headingController =
        new SQPIDController(headingCoeffs.kP, headingCoeffs.kI, headingCoeffs.kD);

    // Set heading controller to wrap around ±π
    this.headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SQPIDHolonomicFollower(
      PIDCoefficients axialCoeffs, PIDCoefficients lateralCoeffs, PIDCoefficients headingCoeffs) {
    this(axialCoeffs, lateralCoeffs, headingCoeffs, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
  }

  @Override
  public void followTrajectory(Trajectory trajectory) {
    axialController.reset();
    lateralController.reset();
    headingController.reset();

    super.followTrajectory(trajectory);
  }

  @NonNull
  @Override
  public Pose2d getLastError() {
    return lastError;
  }

  @Override
  protected void setLastError(@NonNull Pose2d lastError) {
    this.lastError = lastError;
  }

  @NonNull
  @Override
  protected DriveSignal internalUpdate(
      @NonNull Pose2d currentPose, @Nullable Pose2d currentRobotVel) {
    double t = elapsedTime();

    Pose2d targetPose = trajectory.get(t);
    Pose2d targetVel = trajectory.velocity(t);
    Pose2d targetAccel = trajectory.acceleration(t);

    Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);
    Pose2d targetRobotAccel =
        Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel);

    Pose2d poseError = Kinematics.calculateRobotPoseError(targetPose, currentPose);

    // Set the setpoint as the error and use 0 as measurement for SQPID control
    //    axialController.setSetpoint(poseError.getX());
    //    lateralController.setSetpoint(poseError.getY());
    //    headingController.setSetpoint(poseError.getHeading());

    // Update velocity targets for feed-forward
    double currentVelX = currentRobotVel != null ? currentRobotVel.getX() : 0.0;
    double currentVelY = currentRobotVel != null ? currentRobotVel.getY() : 0.0;
    double currentVelHeading = currentRobotVel != null ? currentRobotVel.getHeading() : 0.0;

    // Calculate corrections using SQPID
    double axialCorrection = axialController.calculate(0.0, poseError.getX(), currentVelX);
    double lateralCorrection = lateralController.calculate(0.0, poseError.getY(), currentVelY);
    double headingCorrection =
        headingController.calculate(0.0, poseError.getHeading(), currentVelHeading);

    // Combine feed-forward and feedback
    Pose2d correctedVelocity =
        new Pose2d(
            targetRobotVel.getX() + axialCorrection,
            targetRobotVel.getY() + lateralCorrection,
            targetRobotVel.getHeading() + headingCorrection);

    setLastError(poseError);

    return new DriveSignal(correctedVelocity, targetRobotAccel);
  }
}
