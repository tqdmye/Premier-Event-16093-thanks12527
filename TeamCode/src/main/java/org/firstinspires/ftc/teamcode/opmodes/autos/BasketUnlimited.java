package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Basket ∞", group = "Autos")
public class BasketUnlimited extends AutoCommandBase {
  // For Basket Scoring
  public static Pose2dHelperClass Basket = new Pose2dHelperClass(-56, -56, 45);

  public static Pose2dHelperClass BasketForSpline = new Pose2dHelperClass(-59, -54.5, 60);

  public static Pose2dHelperClass S1Basket =
      new Pose2dHelperClass(-60.124, -54, Math.toDegrees(1.1506));

  public static Pose2dHelperClass PreloadBasket =
      new Pose2dHelperClass(-58, -53, Math.toDegrees(1.1506));

  public static Pose2dHelperClass S2Basket =
      new Pose2dHelperClass(-64.694, -53, Math.toDegrees(1.621));

  // The right sample
  public static Pose2dHelperClass S1 = new Pose2dHelperClass(-59.7, -48.5, Math.toDegrees(1.12748));

  // The middle sample
  public static Pose2dHelperClass S2 = new Pose2dHelperClass(-63, -50.8168, Math.toDegrees(1.4281));

  // The left sample
  public static Pose2dHelperClass S3 = new Pose2dHelperClass(-61.4, -48.8, 113);

  // Middle point for spline
  public static Pose2dHelperClass splinePoint1 = new Pose2dHelperClass(-24.5, -5, 0);

  public static Pose2dHelperClass splinePoint2 = new Pose2dHelperClass(-24.5, 0, 0);

  public static long basketWaitMs = 630;

  public static long basketWaitForAutoPickMs = 0;

  public static long firstBasketWaitMs = 520;

  public static long pick2Handoff = 0;

  public static long startStowToPath = 200;

  public static long stowedToGrabDelay = 200;

  public static long stopLinearToLLM = 0;

  public static long S3SlideDelay = 0;

  public static double S1TurnPos = 0.35;
  public static double S2TurnPos = 0.25;
  public static double S3TurnPos = 0.17;

  public static Pose2d startPose = new Pose2d(-40.13, -63.82, Math.toRadians(90));

  @Override
  public Pose2d getStartPose() {
    return startPose;
  }

  @Override
  public void initializeSuperStructure() {
    drive.setPoseEstimate(startPose);
    drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.SLOW);
    drive.breakFollowing(true);
    slide.stow();
    slide.openIntakeClaw();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
    vision.initializeCamera();
    vision.setLEDPWM();
  }

  // spotless:off
  @Override
  public Command runAutoCommand() {
    Supplier<Command> slideExtendCommand =
        () -> new InstantCommand(() -> slide.forwardSlideExtension());

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              drive.setPoseEstimate(startPose);
              drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.SLOW);
            }),
        new LineToLinearPathCommand(drive, PreloadBasket.toPose2d())
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S1TurnPos))),
                upLiftToBasket()),
        stowArmFromBasket().alongWith(slideExtendCommand.get()),
        new WaitCommand(200),
        new LineToLinearPathCommand(drive, S1.toPose2d()),
        slide.grabCommand(),
        //            .alongWith(
        //                new WaitUntilCommand(() -> lift.atHome(15))
        //                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, S1Basket.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        new WaitCommand(basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S2TurnPos))),
                slideExtendCommand.get()),
        new LineToLinearPathCommand(drive, S2.toPose2d()),
        slide.grabCommand(),
        //            .alongWith(
        //                new WaitUntilCommand(() -> lift.atHome(15))
        //                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, S1Basket.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        new WaitCommand(basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S3TurnPos))),
                new WaitCommand(S3SlideDelay).andThen(slideExtendCommand.get())),
        new LineToLinearPathCommand(drive, S3.toPose2d()),
        slide.grabCommand(),
        //            .alongWith(
        //                new WaitUntilCommand(() -> lift.atHome(15))
        //                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, BasketForSpline.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        new WaitCommand(basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slowHandoff(),
                new WaitCommand(startStowToPath)
                    .andThen(
                        new LineToLinearPathCommand(drive, splinePoint1.toPose2d())
                            .andThen(new WaitCommand(stopLinearToLLM)))),

        // new SplineToPathCommand(drive, splinePoint1.toPose2d(), goPickTangent),
        //        wait(drive, 100),
        autoSamplePickCommand(splinePoint1.toPose2d()),
        new LineToLinearPathCommand(drive, BasketForSpline.toPose2d())
            .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
        wait(drive, basketWaitForAutoPickMs),
        stowArmFromBasket()
            .alongWith(
                slowHandoff(),
                new WaitCommand(startStowToPath)
                    .andThen(
                        new LineToLinearPathCommand(drive, splinePoint2.toPose2d())
                            .andThen(new WaitCommand(stopLinearToLLM)))),
        // new SplineToPathCommand(drive, splinePoint2.toPose2d(), goPickTangent),
        //        wait(drive, 100),
        autoSamplePickCommand(splinePoint1.toPose2d()),
        new LineToLinearPathCommand(drive, BasketForSpline.toPose2d())
            .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
        wait(drive, basketWaitForAutoPickMs),
        stowArmFromBasket());
  }
  // spotless:
}
