package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

public class SampleAutoAlignCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private final Vision vision;
  private final Telemetry telemetry;

  private AtomicReference<Double> turnServo;
  private AtomicReference<Double> slideExtension;

  private final double tickPerUnit = 422 / (440 / 25.4); // tick per inches
  private TrajectorySequence trajectorySequence;
  private boolean isTargetVisibleWhenStart = true;

  @Getter private boolean isInitializing = true;

  public SampleAutoAlignCommand(
      SampleMecanumDrive drive,
      Vision vision,
      Telemetry telemetry,
      AtomicReference<Double> turnServoSupplier,
      AtomicReference<Double> slideExtensionSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.telemetry = telemetry;
    this.turnServo = turnServoSupplier;
    this.slideExtension = slideExtensionSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    isInitializing = true;

    isTargetVisibleWhenStart = vision.isTargetVisible();

    setTurnServo();
    telemetry.addData("isVisibleWhenStart", isTargetVisibleWhenStart);

    Pose2d currentPoseRelativeToField = drive.getPoseEstimate();
    telemetry.addData("Current Pose", currentPoseRelativeToField);

    double distanceOffset = vision.getDistance() / 25.4; // inches
    double slideExtensionValue = 0;

    if (distanceOffset > 0) {
      slideExtensionValue = distanceOffset * tickPerUnit;
      distanceOffset = 0;
    }

    telemetry.addData("Slide Extension Value Auto", slideExtensionValue);

    slideExtension.set(slideExtensionValue);

    Pose2d targetPoseRelativeToRobot =
        new Pose2dHelperClass(distanceOffset, -vision.getStrafeOffset() / 25.4, 0).toPose2d();
    telemetry.addData("Target Robot Pose", targetPoseRelativeToRobot);

    // Transform target pose from robot-relative to field-relative coordinates
    double fieldX =
        currentPoseRelativeToField.getX()
            + (targetPoseRelativeToRobot.getX() * Math.cos(currentPoseRelativeToField.getHeading())
                - targetPoseRelativeToRobot.getY()
                    * Math.sin(currentPoseRelativeToField.getHeading()));
    double fieldY =
        currentPoseRelativeToField.getY()
            + (targetPoseRelativeToRobot.getX() * Math.sin(currentPoseRelativeToField.getHeading())
                + targetPoseRelativeToRobot.getY()
                    * Math.cos(currentPoseRelativeToField.getHeading()));
    double fieldHeading =
        currentPoseRelativeToField.getHeading() + 0; // We move our claw instead of robot heading

    Pose2d targetPoseRelativeToField = new Pose2d(fieldX, fieldY, fieldHeading);
    telemetry.addData("Target Field Pose", targetPoseRelativeToField);
    if (isTargetVisibleWhenStart) {
      trajectorySequence =
          TrajectoryManager.trajectorySequenceBuilder(currentPoseRelativeToField)
              .lineToLinearHeading(targetPoseRelativeToField)
              .build();
    } else {
      cancel();
    }
    drive.followTrajectorySequenceAsync(trajectorySequence);
  }

  public void setTurnServo() {
    Double sampleDegrees = vision.getTurnServoDegree();
    if (sampleDegrees == null) return;

    double mappedDegrees = sampleDegrees;
    if (mappedDegrees > 180) {
      mappedDegrees = 360 - mappedDegrees;
    }
    mappedDegrees = 180 - mappedDegrees;
    double resultDegrees = MathUtils.linear(mappedDegrees, 0, 180, 0.2, 1);
    turnServo.set(resultDegrees);
    telemetry.addData("Result Degrees", resultDegrees);
  }

  @Override
  public void execute() {
    isInitializing = false;
    drive.update();
  }

  @Override
  public void end(boolean interrupted) {
    isTargetVisibleWhenStart = true;
  }

  @Override
  public boolean isFinished() {
    if (!isTargetVisibleWhenStart) {
      return true;
    }
    return !drive.isBusy();
  }
}
