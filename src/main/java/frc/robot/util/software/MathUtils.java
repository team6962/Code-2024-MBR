package frc.robot.util.software;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Some math utility functions for the swerve drive and input processing.
*/
public final class MathUtils {
    /**
     * Math for controlling the swerve drive.
    */
    public static final class SwerveMath {
    /**
     * Finds the distance in radians between specified angles.
     * @return The number of radians between the two angles, from 0 to π.
    */
    public static double angleDistance(double alpha, double beta) {
      // Calculates the difference between the angles in a range from 0 to 2π.
      double phi = Math.abs(beta - alpha) % (2.0 * Math.PI);

      // Returns the distance from phi to 0 or 2pi, whichever is smaller. Equivalent
      // to min(phi, 2π - phi) and π - abs(phi - π).
      return phi > Math.PI ? (2.0 * Math.PI) - phi : phi;
    }

    /**
     * Logical inverse of the Pose exponential from 254. Taken from team 3181.
     *
     * @param transform Pose to perform the log on.
     * @return {@link Twist2d} of the transformed pose.
     */
    public static Twist2d PoseLog(final Pose2d transform) {
      final double kEps          = 1E-9;
      final double dtheta        = transform.getRotation().getRadians();
      final double half_dtheta   = 0.5 * dtheta;
      final double cos_minus_one = transform.getRotation().getCos() - 1.0;
      double       halftheta_by_tan_of_halfdtheta;
      if (Math.abs(cos_minus_one) < kEps) {
        halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
      } else {
        halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
      }
      final Translation2d translation_part = transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
      return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }
  }

  /**
     * Math for taking input from the controller.
    */
  public static final class InputMath {
    public static double addLinearDeadband(double input, double deadband) { // input ranges from -1 to 1
      if (Math.abs(input) <= deadband) return 0.0;
      if (input > 0) return map(input, deadband, 1.0, 0.0, 1.0);
      return map(input, -deadband, -1.0, 0.0, -1.0);
    }

    public static double mapBothSides(double X, double A, double B, double C, double D) {
      if (X > 0.0) return map(X, A, B, C, D);
      if (X < 0.0) return map(X, -A, -B, -C, -D);
      return 0.0;
    }

    public static Translation2d addCircularDeadband(Translation2d input, double deadband) {
      double magnitude = input.getNorm();
      double direction = input.getAngle().getRadians();
      if (Math.abs(magnitude) <= deadband) return new Translation2d();
      magnitude = nonLinear(map(magnitude, deadband, 1.0, 0.0, 1.0));
      return new Translation2d(magnitude * Math.cos(direction), magnitude * Math.sin(direction));
    }

    public static double nonLinear(double x) {
      return (1 - Math.cos(Math.abs(x) * Math.PI / 2.0)) * Math.signum(x);
    }
  }

  /**
   * Maps a value from one range to another.
   * @param X The value to map.
   * @param A The lower bound of the value's current range.
   * @param B The upper bound of the value's current range.
   * @param C The lower bound of the value's target range.
   * @param D The upper bound of the value's target range.
   * @return X, mapped from [A, B] to [C, D].
   */
  public static double map(double X, double A, double B, double C, double D) {
    return (X - A) / (B - A) * (D - C) + C;
  }

  /**
   * Computes the modulus of a number rounding down instead of towards 0.
   * Also equivalent to x - floor(x / r) * r.
   * @param x The number to mod.
   * @param r The modulus.
   * @return x mod r, rounded down.
  */
  public static double mod(double x, double r) {
    return ((x % r) + r) % r;
  }

  public static boolean isIdle(XboxController xboxController) {
    if (Math.abs(xboxController.getRawAxis(0)) > 0.5) {
      return false;
    }
    if (Math.abs(xboxController.getRawAxis(1)) > 0.5) {
      return false;
    }
    return true;
  }

  // public static double trangleArea(Translation2d p1, Translation2d p2, Translation2d p3) {
  //   return Math.abs((p1.getX() * (p2.getY() - p3.getY()) + p2.getX() * (p3.getY() - p1.getY()) + p3.getX() * (p1.getY() - p2.getY())) / 2.0);
  // }
  public static boolean isInsideTriangle(Translation2d A, Translation2d B, Translation2d C, Translation2d P) {
    // Calculate the barycentric coordinates
    // of point P with respect to triangle ABC
    double denominator = ((B.getY() - C.getY()) * (A.getX() - C.getX()) +
                          (C.getX() - B.getX()) * (A.getY() - C.getY()));
    double a = ((B.getY() - C.getY()) * (P.getX() - C.getX()) +
                (C.getX() - B.getX()) * (P.getY() - C.getY())) / denominator;
    double b = ((C.getY() - A.getY()) * (P.getX() - C.getX()) +
                (A.getX() - C.getX()) * (P.getY() - C.getY())) / denominator;
    double c = 1 - a - b;
 
    // Check if all barycentric coordinates
    // are non-negative
    if (a >= 0 && b >= 0 && c >= 0) {
      return true;
    } else {
      return false;
    }
  }
}
