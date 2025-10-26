package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/** AdvantageKit IO layer for vision measurements. */
public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean isConnected;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public TargetTransform[] targetTransforms = new TargetTransform[0];
  }

  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {}

  record TargetTransform(
      double timestamp,
      int fiducialId,
      Transform3d cameraToTarget,
      double ambiguity,
      double distance) {}

  default void updateInputs(VisionIOInputs inputs) {}
}
