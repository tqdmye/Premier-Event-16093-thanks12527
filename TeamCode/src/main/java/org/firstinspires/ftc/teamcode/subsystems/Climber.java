package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Climber extends SubsystemBase {
  public static long climberUpMs = 2000;
  private final DcMotorEx climber;

  public Climber(final HardwareMap hardwareMap) {
    climber = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
    climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    climber.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void elevate() {
    climber.setPower(1);
  }

  public void decline() {
    climber.setPower(-1);
  }

  public void stop() {
    climber.setPower(0);
  }

  public void keep() {
    climber.setPower(-0.2);
  }

  public void holdOn() {
    climber.setPower(-0.5);
  }

  public Command elevateCommand() {
    return new StartEndCommand(this::elevate, this::stop);
  }

  public Command declineCommand() {
    return new StartEndCommand(this::decline, this::stop);
  }

  public Command holdOnCommand() {
    return new StartEndCommand(this::holdOn, this::stop);
  }
}
