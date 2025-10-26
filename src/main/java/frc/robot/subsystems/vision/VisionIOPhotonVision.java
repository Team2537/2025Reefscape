package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import lib.math.geometry.FieldConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Real/sim shared PhotonVision implementation. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isConnected = camera.isConnected();

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    List<TargetTransform> targetTransforms = new ArrayList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      for (PhotonTrackedTarget target : result.getTargets()) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        double distance = cameraToTarget.getTranslation().getNorm();
        targetTransforms.add(
            new TargetTransform(
                result.getTimestampSeconds(),
                target.getFiducialId(),
                cameraToTarget,
                target.getPoseAmbiguity(),
                distance));
      }

      result
          .getMultiTagResult()
          .ifPresentOrElse(
              multitagResult -> {
                Transform3d fieldToCamera = multitagResult.getEstimatedPose().best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (PhotonTrackedTarget target : result.getTargets()) {
                  totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }

                tagIds.addAll(multitagResult.getFiducialIDsUsed());

                poseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.getEstimatedPose().ambiguity,
                        multitagResult.getFiducialIDsUsed().size(),
                        result.getTargets().isEmpty()
                            ? 0.0
                            : totalTagDistance / result.getTargets().size()));
              },
              () -> {
                if (!result.getTargets().isEmpty()) {
                  PhotonTrackedTarget target = result.getTargets().get(0);
                  FieldConstants.TAG_LAYOUT
                      .getTagPose(target.getFiducialId())
                      .ifPresent(
                          tagPose -> {
                            Transform3d fieldToTarget =
                                new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                            Transform3d cameraToTarget = target.getBestCameraToTarget();
                            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                            Pose3d robotPose =
                                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                            tagIds.add(target.getFiducialId());
                            poseObservations.add(
                                new PoseObservation(
                                    result.getTimestampSeconds(),
                                    robotPose,
                                    target.getPoseAmbiguity(),
                                    1,
                                    cameraToTarget.getTranslation().getNorm()));
                          });
                }
              });
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.targetTransforms = targetTransforms.toArray(new TargetTransform[0]);
  }
}
