package lib.math.geometry;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Utility methods for mirroring field objects between alliances. */
public final class FieldFlipUtil {
  private FieldFlipUtil() {}

  public static Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  public static Translation2d flip(Translation2d translation) {
    double x = (2.0 * FieldConstants.FIELD_CENTER.getX()) - translation.getX();
    double y = (2.0 * FieldConstants.FIELD_CENTER.getY()) - translation.getY();
    return new Translation2d(x, y);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        flip(pose.getTranslation()), pose.getRotation().plus(Rotation2d.fromDegrees(180.0)));
  }

  public static Translation3d flip(Translation3d translation) {
    double x = (2.0 * FieldConstants.FIELD_CENTER.getX()) - translation.getX();
    double y = (2.0 * FieldConstants.FIELD_CENTER.getY()) - translation.getY();
    return new Translation3d(x, y, translation.getZ());
  }

  public static Pose3d flip(Pose3d pose) {
    Translation3d flippedTranslation = flip(pose.getTranslation());
    Rotation3d flippedRotation =
        pose.getRotation().rotateBy(new Rotation3d(Rotation2d.fromDegrees(180.0)));
    return new Pose3d(flippedTranslation, flippedRotation);
  }

  public static Rectangle2d flip(Rectangle2d rectangle) {
    return new Rectangle2d(flip(rectangle.getCenter()), rectangle.getXWidth(), rectangle.getYWidth());
  }

  public static Ellipse2d flip(Ellipse2d ellipse) {
    return new Ellipse2d(flip(ellipse.getCenter()), ellipse.getXSemiAxis(), ellipse.getYSemiAxis());
  }
}
