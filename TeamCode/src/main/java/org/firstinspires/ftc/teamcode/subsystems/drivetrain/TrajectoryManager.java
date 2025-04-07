package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectoryManager {
  private static final TrajectoryVelocityConstraint VEL_CONSTRAINT =
      getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
  private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT =
      getAccelerationConstraint(MAX_ACCEL);

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

  public static TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
    return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public static TrajectoryBuilder trajectoryBuilder(
      Pose2d startPose, double maxVel, double maxAccel) {
    return new TrajectoryBuilder(
        startPose,
        getVelocityConstraint(maxVel, MAX_ANG_VEL, TRACK_WIDTH),
        getAccelerationConstraint(maxAccel));
  }

  public static TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
    return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public static TrajectoryBuilder trajectoryBuilder(
      Pose2d startPose,
      boolean reversed,
      double velConstraint,
      double angVelConstraint,
      double accelConstraint) {
    return new TrajectoryBuilder(
        startPose,
        reversed,
        getVelocityConstraint(velConstraint, angVelConstraint, TRACK_WIDTH),
        getAccelerationConstraint(accelConstraint));
  }

  public static TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
    return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
  }

  public static TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
    return new TrajectorySequenceBuilder(
        startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL);
  }
}
