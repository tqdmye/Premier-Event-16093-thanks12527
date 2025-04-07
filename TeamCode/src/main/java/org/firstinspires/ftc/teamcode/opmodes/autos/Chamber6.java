package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Chamber 6", group = "Autos")
public class Chamber6 extends AutoCommandBase {
  public static Pose2d startPose = new Pose2d(38.9 - 23.75, -61.26, Math.toRadians(270));

  public static Pose2dHelperClass grabSpecPose = new Pose2dHelperClass(38.9, -62.26, 270);
  public static Pose2dHelperClass grabSpecStartPose = new Pose2dHelperClass(38.9, -61.26, 270);

  public static Pose2dHelperClass spec1Pose = new Pose2dHelperClass(5, -28.5, 270);

  public static Pose2dHelperClass spec2Pose = new Pose2dHelperClass(3, -28.5, 270);

  public static Pose2dHelperClass spec3Pose = new Pose2dHelperClass(1, -28.5, 270);

  public static Pose2dHelperClass spec4Pose = new Pose2dHelperClass(-1, -28.5, 270);

  public static Pose2dHelperClass spec5Pose = new Pose2dHelperClass(-3, -28.5, 270);

  public static Pose2dHelperClass spec6Pose = new Pose2dHelperClass(-1.118, -31.49, 270);

  public static long delayToUpLift = 100;
  public static long delayToStow = 0;
  public static long hangToOpenClaw = 100;
  public static long pathToGrab = 100;
  public static long liftUpToOpen = 100;
  public static long pathStartToSwipe = 100;
  public static double turnDegrees = 100;

  public static long waitForArm = 200;

  private final Trajectory sampleToObservation =
      TrajectoryManager.trajectoryBuilder(new Pose2d(3, -30, Math.toRadians(270)), 40, 25)
          .splineToLinearHeading(new Pose2d(37, -25, Math.toRadians(270)), Math.toRadians(90))
          .splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(270)), Math.toRadians(0))
          .splineToLinearHeading(new Pose2d(45, -25, Math.toRadians(270)), Math.toRadians(270))
          .splineToLinearHeading(new Pose2d(46, -48, Math.toRadians(270)), Math.toRadians(90))
          .splineToLinearHeading(new Pose2d(46, -25, Math.toRadians(270)), Math.toRadians(90))
          .splineToLinearHeading(new Pose2d(51, -10, Math.toRadians(270)), Math.toRadians(0))
          .splineToLinearHeading(new Pose2d(55, -25, Math.toRadians(270)), Math.toRadians(270))
          .splineToLinearHeading(new Pose2d(56, -48, Math.toRadians(270)), Math.toRadians(90))
          .splineToLinearHeading(new Pose2d(56, -25, Math.toRadians(270)), Math.toRadians(90))
          .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(270)), Math.toRadians(270))
          .splineToLinearHeading(new Pose2d(62, -25, Math.toRadians(270)), Math.toRadians(270))
          .splineToLinearHeading(new Pose2d(62, -48, Math.toRadians(270)), Math.toRadians(270))
          .build();

  @Override
  public Pose2d getStartPose() {
    return startPose;
  }

  @Override
  public void initializeSuperStructure() {
    drive.setPoseEstimate(startPose);
    drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
    drive.breakFollowing(true);
    liftClaw.grabFromWall();
    slide.foldSlideStructure();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
  }

  @Override
  public Command runAutoCommand() {

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              drive.setPoseEstimate(startPose);
              drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
            }),
        liftClaw.closeClawCommand(),
        new LineToLinearPathCommand(drive, spec1Pose.toPose2d())
            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
        hangSpecimen(),
        new WaitUntilCommand(() -> lift.isPreHang()),
        liftClaw.openClawCommand(),
        new AutoDriveCommand(drive, sampleToObservation)
            .alongWith(new WaitCommand(delayToStow).andThen(stowLiftAndArm())),
        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d()),
        specimenCycle(spec2Pose.toPose2d()),
        new WaitCommand(pathToGrab),
        specimenCycle(spec3Pose.toPose2d()),
        new WaitCommand(pathToGrab),
        specimenCycle(spec4Pose.toPose2d()),
        new WaitCommand(pathToGrab),
        specimenCycle(spec5Pose.toPose2d()),
        new WaitCommand(pathToGrab),
        // specimenCycle(spec6Pose.toPose2d()));
        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d()));
  }

  public Command hangSpecimen() {
    return new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG));
  }

  public Command stowLiftAndArm() {
    return new SequentialCommandGroup(
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 100),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command switchToSpecimenMode() {
    return new SequentialCommandGroup(
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.AVOID_COLLISION, 100),
        slide.foldSlideStructureCommand(),
        new WaitCommand(200),
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 0),
        liftClaw.openClawCommand());
  }

  public Command specimenCycle(Pose2d specHangPose) {
    return new SequentialCommandGroup(
        liftClaw.closeClawCommand(),
        new LineToLinearPathCommand(drive, specHangPose)
            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
        hangSpecimen(),
        new WaitUntilCommand(() -> lift.isPreHang()),
        liftClaw.openClawCommand(),
        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
            .alongWith(new WaitCommand(delayToStow).andThen(stowLiftAndArm())));
  }

  // slide Arm 0.22 wrist 0.6

  private Command turnCommand(double turnDegrees) {
    return new FunctionalCommand(
        () -> drive.turnAsync(Math.toRadians(turnDegrees)),
        () -> drive.update(),
        (interrupted) -> {},
        () ->
            !drive.isBusy()
                || MathUtils.isNear(
                    turnDegrees,
                    drive.getPoseEstimate().getHeading(),
                    DriveConstants.headingPoseError),
        drive);
  }
}
