package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import edu.wpi.first.math.MathUtil;
import lombok.RequiredArgsConstructor;

@Config
public class LiftClaw extends SubsystemBase {
  public static double LiftArmServo_SCORE_BASKET = 0.7;
  public static double LiftArmServo_STOW = 0.165;
  public static double LiftArmServo_GRAB_WALL = 0.26;
  public static double LiftArmServo_SCORE_CHAMBER = 0.7;
  public static double LiftArmServo_AVOID_COLLISION = 0.4;

  public static double ClawServo_CLOSE = 0.68;
  public static double ClawServo_OPEN = 0.28;
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private boolean isClawOpen;

  public LiftClaw(final HardwareMap hardwareMap) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "clawServo"); // 0 Close 0.5 Open
    liftClawServo.setPosition(LiftArmServo_STOW);
    setServoController(true);
  }

  public void switchLiftClaw() {
    if (isClawOpen) {
      liftClawServo.setPosition(ClawServo_CLOSE);
    } else {
      liftClawServo.setPosition(ClawServo_OPEN);
    }
    isClawOpen = !isClawOpen;
  }

  public void openClaw() {
    isClawOpen = true;
    liftClawServo.setPosition(ClawServo_OPEN);
  }

  public void closeClaw() {
    isClawOpen = false;
    liftClawServo.setPosition(ClawServo_CLOSE);
  }

  public Command openClawCommand() {
    return new InstantCommand(this::openClaw);
  }

  public Command closeClawCommand() {
    return new InstantCommand(this::closeClaw);
  }

  public Command switchClawCommand() {
    return new InstantCommand(this::switchLiftClaw);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(LiftArmServo_STOW);
  }

  public void grabFromWall() {
    liftArmServo.setPosition(LiftArmServo_GRAB_WALL);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(LiftArmServo_SCORE_BASKET);
  }

  public Command setLiftClawServo(LiftClawState state, long delay) {
    return setServoPosCommand(liftArmServo, state.servoPos, delay);
  }

  public boolean isLiftArmStow() {
    return MathUtil.isNear(LiftArmServo_STOW, liftArmServo.getPosition(), 0.01);
  }

  public boolean isLiftArmGrab() {
    return MathUtil.isNear(LiftArmServo_GRAB_WALL, liftArmServo.getPosition(), 0.01);
  }

  private Command setServoPosCommand(Servo servo, double pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  servo.setPosition(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> servo.getPosition() != pos);
  }

  @RequiredArgsConstructor
  public enum LiftClawState {
    STOW(LiftArmServo_STOW),
    GRAB_FROM_WALL(LiftArmServo_GRAB_WALL),
    SCORE_BASKET(LiftArmServo_SCORE_BASKET),
    AVOID_COLLISION(LiftArmServo_AVOID_COLLISION),
    SCORE_CHAMBER(LiftArmServo_SCORE_CHAMBER);

    private final double servoPos;
  }

  public void setServoController(boolean enable) {
    if (enable) {
      liftArmServo.getController().pwmEnable();
      liftClawServo.getController().pwmEnable();
    } else {
      liftArmServo.getController().pwmDisable();
      liftClawServo.getController().pwmDisable();
    }
  }
}
