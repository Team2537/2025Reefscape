package lib.math.geometry;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/** Utility helpers for geometry conversions that replaced Kotlin extensions. */
public final class GeometryUtil {
  private GeometryUtil() {}

  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d(pose.getRotation()));
  }

  public static Pose2d toPose2d(Pose3d pose) {
    return pose.toPose2d();
  }

  public static List<Pose2d> getCorners(Rectangle2d rectangle) {
    double x = rectangle.getCenter().getX();
    double y = rectangle.getCenter().getY();
    double halfWidth = rectangle.getXWidth() / 2.0;
    double halfHeight = rectangle.getYWidth() / 2.0;
    Rotation2d rotation = rectangle.getRotation();
    return List.of(
        new Pose2d(x - halfWidth, y - halfHeight, rotation),
        new Pose2d(x + halfWidth, y - halfHeight, rotation),
        new Pose2d(x + halfWidth, y + halfHeight, rotation),
        new Pose2d(x - halfWidth, y + halfHeight, rotation));
  }

  public static List<Pose2d> getCardinals(Ellipse2d ellipse) {
    double x = ellipse.getCenter().getX();
    double y = ellipse.getCenter().getY();
    double a = ellipse.getXSemiAxis();
    double b = ellipse.getYSemiAxis();

    double[] directions = {
      Math.PI / 2,
      Math.PI / 4,
      0.0,
      -Math.PI / 4,
      -Math.PI / 2,
      -3 * Math.PI / 4,
      Math.PI,
      3 * Math.PI / 4
    };

    List<Pose2d> poses = new ArrayList<>(directions.length);
    for (double theta : directions) {
      double dx = a * Math.cos(theta);
      double dy = b * Math.sin(theta);
      poses.add(new Pose2d(x + dx, y + dy, new Rotation2d(Math.atan2(-dy, -dx))));
    }
    return poses;
  }

  public static Pose2d nudge(Pose2d pose, double dxMeters, double dyMeters) {
    return nudge(pose, dxMeters, dyMeters, new Rotation2d());
  }

  public static Pose2d nudge(Pose2d pose, double dxMeters, double dyMeters, Rotation2d rotation) {
    return pose.transformBy(new Transform2d(new Translation2d(dxMeters, dyMeters), rotation));
  }
}
