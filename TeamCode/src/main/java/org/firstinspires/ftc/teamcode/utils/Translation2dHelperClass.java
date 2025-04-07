package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import lombok.Getter;

public class Translation2dHelperClass {
  @Getter public double X, Y;

  public Translation2dHelperClass(double x, double y) {
    this.X = x;
    this.Y = y;
  }

  public Translation2d toTranslation2d() {
    return new Translation2d(X, Y);
  }

  public Vector2d toVector2d() {
    return new Vector2d(X, Y);
  }
}
