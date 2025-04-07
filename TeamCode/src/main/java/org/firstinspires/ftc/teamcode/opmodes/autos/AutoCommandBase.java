package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SampleAutoAlignCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStructure;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

/**
 * Layout: Field Coordinate: +-----+-----+-----+-----+-----+-----^ x | 0 | 1 | 2 | 3 | 4 | 5 |
 * <-----+-----+-----+-----+-----+-----+ CCW+ y O Robot Coordinate:
 *
 * <p>Center
 */
@Config
public abstract class AutoCommandBase extends LinearOpMode {
  protected LiftClaw liftClaw;
  protected Lift lift;
  protected SlideSuperStructure slide;
  protected SampleMecanumDrive drive;
  protected Climber climb;
  protected Vision vision;
  protected Pose2d currentPose = new Pose2d();

  public static long handoff_slide2LiftCloseDelayMs = 150;
  public static long handoff_liftClose2OpenIntakeDelayMs = 100;
  public static int liftClawScoreThreshold = 37;

  private static TrajectorySequence sequence = null;

  @Getter private static Pose2d autoEndPose = new Pose2d();

  protected void initialize() {
    initialize(true);
  }

  protected void initialize(boolean telemetryInDashboard) {
    if (telemetryInDashboard) {
      this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    CommandScheduler.getInstance().reset();
    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    climb = new Climber(hardwareMap);
    slide = new SlideSuperStructure(hardwareMap, telemetry);
    drive = new SampleMecanumDrive(hardwareMap, telemetry);
    vision = new Vision(hardwareMap, telemetry);
    // drive.resetPinpoint();
  }

  protected Command upLiftToBasket() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HIGH_BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > liftClawScoreThreshold)
            .andThen(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, 0)));
  }

  protected Command followTrajectory(TrajectorySequence trajectorySequence) {
    return new AutoDriveCommand(drive, trajectorySequence);
  }

  protected Command autoSamplePickCommand(Pose2d goalPose) {
    AtomicReference<Double> turnServoSupplier = new AtomicReference<>();
    AtomicReference<Double> slideExtensionSupplier = new AtomicReference<>();
    SampleAutoAlignCommand sampleAutoAlignCommand =
        new SampleAutoAlignCommand(
            drive, vision, telemetry, turnServoSupplier, slideExtensionSupplier);
    return new SequentialCommandGroup(
        sampleAutoAlignCommand.alongWith(
            new WaitUntilCommand(() -> !sampleAutoAlignCommand.isInitializing())
                .andThen(
                    slide.aimCommand(turnServoSupplier),
                    new InstantCommand(() -> slide.forwardSlideExtension(slideExtensionSupplier)))),
        new WaitCommand(50),
        slide.grabCommand());
    //        new ConditionalCommand(
    //            new ScheduleCommand(
    //                new LineToLinearPathCommand(drive, goalPose)
    //                    .andThen(autoSamplePickCommand(goalPose))),
    //            new InstantCommand(),
    //            () -> !slide.isClawGrabSample()));
  }

  protected Command stowArmFromBasket() {
    return new SequentialCommandGroup(
        liftClaw.openClawCommand(),
        new WaitCommand(200),
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 100),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  protected Command slowHandoff() {
    return slowHandoff(slide, liftClaw).beforeStarting(() -> slide.setAutoTurnControl(false));
  }

  public static Command slowHandoff(SlideSuperStructure slide, LiftClaw liftClaw) {
    return slide
        .slowHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(liftClaw.closeClawCommand())
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  protected Command fastHandoff() {
    return fastHandoff(slide, liftClaw).beforeStarting(() -> slide.setAutoTurnControl(false));
  }

  public static Command fastHandoff(SlideSuperStructure slide, LiftClaw liftClaw) {
    return new SequentialCommandGroup(
        liftClaw.openClawCommand(),
        slide.fastHandoffCommand().andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs)),
        liftClaw.closeClawCommand(),
        new WaitCommand(handoff_liftClose2OpenIntakeDelayMs),
        new InstantCommand(slide::openIntakeClaw));
  }

  public Command wait(SampleMecanumDrive drive, long ms) {
    return new ParallelDeadlineGroup(
        new WaitCommand(ms), new RunCommand(drive::update).interruptOn(this::isStopRequested));
  }

  public Command initializeCommand() {
    return new ParallelCommandGroup(
        //        new InstantCommand(slide::forwardSlideExtension),
        liftClaw.closeClawCommand(),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public Command autoFinish() {
    return new ParallelCommandGroup(
        // TODO: needs discussion
        slide.manualResetCommand().withTimeout(1000), // interruptOn(slide::atHome),
        // lift.resetCommand().interruptOn(() -> lift.atHome(3)),
        lift.manualResetCommand().withTimeout(1000),
        liftClaw.openClawCommand(),
        new InstantCommand(() -> autoEndPose = drive.getPoseEstimate()));
  }

  public Command upLiftToHang() {
    return new SequentialCommandGroup(
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_CHAMBER, 200),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)));
  }

  /**
   * Gets the command to run in auto, this should be implemented in each auto.
   *
   * @return The command to run.
   */
  public abstract Command runAutoCommand();

  /**
   * Gets the robot starting pose in field coordinate or its respective coordinates.
   *
   * @return The start pose following RoadRunner's coordinate system.
   */
  public abstract Pose2d getStartPose();

  public abstract void initializeSuperStructure();

  @Override
  public void runOpMode() throws InterruptedException {
    initialize();
    initializeSuperStructure();

    drive.setPoseEstimate(getStartPose());
    telemetry.addData("Init Complete", "Start Now");
    Command toRun = runAutoCommand().andThen(autoFinish());
    waitForStart();

    CommandScheduler.getInstance().schedule(toRun);

    while (opModeIsActive() && !isStopRequested()) {
      currentPose = drive.getPoseEstimate();
      lift.periodicAsync();
      CommandScheduler.getInstance().run();
      telemetry.addData("Y Velocity", drive.getPoseVelocity().getY());
      telemetry.update();
    }

    if (isStopRequested()) {
      drive.resetHeading();
    }
  }
}
