package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

public class TeleopDriveCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotate;
  private final DoubleSupplier fun;
  private final BooleanSupplier shouldReset;
  private final BooleanSupplier shouldSlow;

  public TeleopDriveCommand(
      SampleMecanumDrive drive,
      DoubleSupplier forward,
      DoubleSupplier fun,
      DoubleSupplier rotate,
      BooleanSupplier shouldReset,
      BooleanSupplier shouldSlow) {
    this.drive = drive;
    this.forward = forward;
    this.rotate = rotate;
    this.fun = fun;
    this.shouldReset = shouldReset;
    this.shouldSlow = shouldSlow;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    if (shouldReset.getAsBoolean()) {
      drive.resetHeading();
    }

    double forwardValue = forward.getAsDouble();
    double funValue = fun.getAsDouble();
    double rotateValue = rotate.getAsDouble() * 0.8;

    if (shouldSlow.getAsBoolean()) {
      forwardValue *= 0.3;
      funValue *= 0.3;
      rotateValue *= 0.3;
    }

    drive.setFieldRelativeDrivePower(new Pose2d(forwardValue, funValue, rotateValue));
  }
}
