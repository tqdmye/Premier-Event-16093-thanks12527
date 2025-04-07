package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FeedBackServo {
  private final Servo servo;
  private final AbsoluteAnalogEncoder analogInput;

  public FeedBackServo(final HardwareMap hardwareMap, String servoName, String analogName) {
    servo = hardwareMap.get(Servo.class, servoName);
    analogInput = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, analogName));
  }

  public double getServoPosition() {
    return analogInput.getCurrentPosition();
  }

  public void setPosition(double position) {
    servo.setPosition(position);
  }

  public double getServoSetpoint() {
    return servo.getPosition();
  }

  public void setOffset(double offset) {
    analogInput.zero(offset);
  }
}
