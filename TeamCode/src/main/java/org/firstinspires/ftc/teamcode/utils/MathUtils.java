package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
  public static boolean isNear(double expected, double actual, double tolerance) {
    if (tolerance < 0) {
      throw new IllegalArgumentException("Tolerance must be a non-negative number!");
    }
    return Math.abs(expected - actual) < tolerance;
  }

  public static double joystickScalar(double num, double min) {
    return joystickScalar(num, min, 0.66, 4);
  }

  private static double joystickScalar(double n, double m, double l, double a) {
    return Math.signum(n) * m
        + (1 - m)
            * (Math.abs(n) > l
                ? Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n)
                : n / a);
  }

  public static double getRadRotDist(double start, double end) {
    double diff = (end - start + Math.PI) % (2 * Math.PI) - Math.PI;
    return diff < -Math.PI ? (diff + (Math.PI * 2)) : diff;
  }

  public static double getRotDist(double start, double end) {
    return MathUtils.getRadRotDist(start, end);
  }

  /**
   * Linear mapping function to map a value from one range to another
   *
   * @param x Input value to map
   * @param inMin Input range minimum
   * @param inMax Input range maximum
   * @param outMin Output range minimum
   * @param outMax Output range maximum
   * @return Mapped value in the output range
   */
  public static double linear(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }
}
