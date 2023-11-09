package frc.robot.util;

/** Contains some basic utility functions that are used often. */
public class Util {
  /** Prevent this class from being instantiated. */
  private Util() {}

  /**
   * limit a value to a given range
   *
   * @param val - Value to check limits on
   * @param maxMagnitude - Magnitude to check limits with
   * @return Value with affected limit
   */
  public static double limit(double val, double maxMagnitude) {
    return limit(val, -maxMagnitude, maxMagnitude);
  }

  /**
   * limit a value to a given range
   *
   * @param val - Value to check limits on
   * @param min - Min value to check lower limit with
   * @param max - Max value to check upper limit with
   * @return Value with affected limit
   */
  public static double limit(double val, double min, double max) {
    return Math.min(max, Math.max(min, val));
  }

  /**
   * Checks if a value is within a given range
   *
   * @param val - Value to check
   * @param maxMagnitude - Magnitude for range to check
   * @return If the value is in the given range
   */
  public static boolean inRange(double val, double maxMagnitude) {
    return inRange(val, -maxMagnitude, maxMagnitude);
  }

  /**
   * Checks if a value is within a given range
   *
   * @param val - Value to check
   * @param min - Min value on range
   * @param max - Max value on range
   * @return If the value is in the given range
   */
  public static boolean inRange(double val, double min, double max) {
    return val > min && val < max;
  }

  /**
   * Checks if a value is closer to a than b
   *
   * @param a - First point
   * @param b - Second point
   * @param val - Value to check
   * @return if a value is closer to a than b
   */
  public static boolean isCloser(double a, double b, double val) {
    return Math.abs(a - val) < Math.abs(b - val);
  }

  /**
   * Linear interpolation between a and b based on percentage alpha
   *
   * @param a - First point
   * @param b - Second value to check
   * @param alpha - Percentage between a and b of desired value
   * @return linear interpolation between a and b based on percentage alpha
   */
  public static double lerp(double a, double b, double alpha) {
    return a * (1.0 - alpha) + b * alpha;
  }

  /**
   * Opposite of linear interpolation for given val between a and b
   *
   * @param a - First point
   * @param b - Second value to check
   * @param alpha - Value between a and b of desired percentage
   * @return percentage that value is between a and b
   */
  public static double unlerp(double a, double b, double value) {
    return (value - a) / (b - a);
  }
}
