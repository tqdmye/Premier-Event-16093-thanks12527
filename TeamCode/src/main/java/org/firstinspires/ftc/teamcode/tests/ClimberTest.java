package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test TeleOp")
public class ClimberTest extends CommandOpMode {
  private DcMotorEx motor;
  private GamepadEx gamepadEx;

  @Override
  public void initialize() {
    gamepadEx = new GamepadEx(gamepad1);
    motor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
    gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenHeld(climbCommand());
  }

  public Command climbCommand() {
    return new CommandBase() {
      @Override
      public void execute() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(1);
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    };
  }
}
