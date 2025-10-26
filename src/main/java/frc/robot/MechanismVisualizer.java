package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Helper for visualizing mechanism poses in AdvantageScope. Replaces the Kotlin singleton.
 */
public final class MechanismVisualizer {
  private static final List<Pose3d> MECHANISM_POSES =
      new ArrayList<>(List.of(new Pose3d(), new Pose3d(), new Pose3d()));

  private MechanismVisualizer() {}

  public static List<Pose3d> getMechanismPoses() {
    return MECHANISM_POSES;
  }

  public static void setArmAngle(Measure<Angle> angle) {
    double angleRadians = angle.in(edu.wpi.first.units.Units.Radians);
    MECHANISM_POSES.set(
        2,
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(12.150963),
                0.0,
                Units.inchesToMeters(15.75)),
            new Rotation3d(0.0, angleRadians, 0.0)));
  }

  public static void updatePoses() {
    Logger.recordOutput("mechanismPoses", MECHANISM_POSES.toArray(Pose3d[]::new));
  }
}
