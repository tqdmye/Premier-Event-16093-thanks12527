package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import java.lang.reflect.Method;

public class ServoPWMControl {
  private final Servo servo;
  private final ServoController controller;
  private final Method changeServoPWMStatus;
  private int ServoPort;

  public ServoPWMControl(Servo servo) {
    this.servo = servo;
    controller = this.servo.getController();
    ServoPort = servo.getPortNumber();
    try {

      changeServoPWMStatus =
          LynxServoController.class.getDeclaredMethod(
              "internalSetPwmEnable", int.class, boolean.class);
      changeServoPWMStatus.setAccessible(true);
    } catch (NoSuchMethodException e) {
      throw new RuntimeException("TM的反射炸了!");
    }
  }

  public void setStatus(boolean enable) {
    try {
      changeServoPWMStatus.invoke(controller, ServoPort, enable);
    } catch (Exception e) {
      throw new RuntimeException("TM的调用方法炸了!");
    }
  }
}
