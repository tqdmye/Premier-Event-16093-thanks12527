package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GeomUtil {
  public static Pose2d toPose2d(Pose2D pose) {
    return new Pose2d(
        pose.getX(DistanceUnit.INCH),
        pose.getY(DistanceUnit.INCH),
        pose.getHeading(AngleUnit.RADIANS));
  }

  public static Pose2D toPose2D(Pose2d pose) {
    return new Pose2D(
        DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
  }
}
