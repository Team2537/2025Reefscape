package lib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Describes how to mirror 2D geometries across the field. Simplified from the Kotlin version to
 * operate directly on standard WPILib classes.
 */
public interface Flipper2d {
  Translation2d flipTranslation(Translation2d translation);

  Rotation2d flipRotation(Rotation2d rotation);

  default Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  default Transform2d flipTransform(Transform2d transform) {
    Translation2d flippedTranslation =
        flipTranslation(new Translation2d(transform.getX(), transform.getY()));
    Rotation2d flippedRotation = flipRotation(transform.getRotation());
    return new Transform2d(flippedTranslation, flippedRotation);
  }

  default Twist2d flipTwist(Twist2d twist) {
    Translation2d flipped = flipTranslation(new Translation2d(twist.dx, twist.dy));
    double flippedTheta = flipRotation(Rotation2d.fromRadians(twist.dtheta)).getRadians();
    return new Twist2d(flipped.getX(), flipped.getY(), flippedTheta);
  }

  Flipper2d NONE = new SimpleFlipper(false, false);
  Flipper2d REFLECT_X = new SimpleFlipper(true, false);
  Flipper2d REFLECT_Y = new SimpleFlipper(false, true);

  /** Simple reflection-based implementation. */
  final class SimpleFlipper implements Flipper2d {
    private final boolean reflectX;
    private final boolean reflectY;
    SimpleFlipper(boolean reflectX, boolean reflectY) {
      this.reflectX = reflectX;
      this.reflectY = reflectY;
    }

    @Override
    public Translation2d flipTranslation(Translation2d translation) {
      double x = translation.getX();
      double y = translation.getY();
      if (reflectX) {
        x = FieldConstants.FIELD_LENGTH_METERS - x;
      }
      if (reflectY) {
        y = FieldConstants.FIELD_WIDTH_METERS - y;
      }
      return new Translation2d(x, y);
    }

    @Override
    public Rotation2d flipRotation(Rotation2d rotation) {
      double radians = rotation.getRadians();
      if (reflectX) {
        radians = -radians;
      }
      if (reflectY) {
        radians = Math.PI - radians;
      }
      return Rotation2d.fromRadians(radians);
    }
  }
}
