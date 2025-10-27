package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lib.math.geometry.FieldConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

/**
 * Vision subsystem responsible for processing pose estimates from PhotonVision.
 */
public final class Vision extends SubsystemBase {
  private static final List<Transform3d> ROBOT_TO_CAMERAS = List.of(
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.801),
              Units.inchesToMeters(11.875000),
              Units.inchesToMeters(6.261304)),
          new Rotation3d(0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(220.0))),
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.801),
              Units.inchesToMeters(-11.875000),
              Units.inchesToMeters(6.261304)),
          new Rotation3d(0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(140.0))));

  private static final double MAX_AMBIGUITY = 0.3;
  private static final double MAX_Z_ERROR = 0.75;
  private static final double LINEAR_STD_DEV_BASELINE = 0.08;
  private static final double ANGULAR_STD_DEV_BASELINE = 0.18;
  private static final double MAX_TRANSLATION_JUMP_METERS = 0.5;
  private static final double MAX_ROTATION_JUMP_RAD = 0.25;
  private static final double MIN_LINEAR_SPEED_MPS = 0.05;

  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final List<VisionIO> ios;
  private final List<VisionIOInputsAutoLogged> inputs;
  private Pose2d lastPose = null;
  private double lastTimestampSec = -1.0;

  public Vision(VisionConsumer consumer) {
    super("vision");
    this.consumer = consumer;
    this.robotPoseSupplier = () -> Robot.getDrive().getPose();
    this.ios = createIOs();
    this.inputs = ios.stream().map(io -> new VisionIOInputsAutoLogged()).toList();
  }

  public List<VisionIOInputsAutoLogged> getInputs() {
    return inputs;
  }

  public List<Transform3d> getRobotToCameras() {
    return ROBOT_TO_CAMERAS;
  }

  public List<VisionIO.TargetTransform> getTargetTransforms() {
    List<VisionIO.TargetTransform> transforms = new ArrayList<>();
    for (VisionIOInputsAutoLogged input : inputs) {
      transforms.addAll(List.of(input.targetTransforms));
    }
    return transforms;
  }

  private List<VisionIO> createIOs() {
    return switch (RobotType.MODE) {
      case REAL ->
        List.of(
            new VisionIOPhotonVision("right_mod_cam", ROBOT_TO_CAMERAS.get(0), 0),
            new VisionIOPhotonVision("left_mod_cam", ROBOT_TO_CAMERAS.get(1), 1));
      case SIMULATION ->
        List.of(
            new VisionIOPhotonVisionSim(
                "right_mod_cam", ROBOT_TO_CAMERAS.get(0), 0, robotPoseSupplier),
            new VisionIOPhotonVisionSim(
                "left_mod_cam", ROBOT_TO_CAMERAS.get(1), 1, robotPoseSupplier));
      case REPLAY -> List.of(new NullVisionIO(), new NullVisionIO());
    };
  }

  @Override
  public void periodic() {
    // Estimate current linear speed from pose delta
    Pose2d currentPose = robotPoseSupplier.get();
    double nowSec = Timer.getFPGATimestamp();
    double linearSpeedMps = 0.0;
    if (lastPose != null && lastTimestampSec > 0.0) {
      double dt = nowSec - lastTimestampSec;
      if (dt > 1e-6) {
        linearSpeedMps = currentPose.getTranslation().getDistance(lastPose.getTranslation()) / dt;
      }
    }

    for (int index = 0; index < ios.size(); index++) {
      VisionIO io = ios.get(index);
      VisionIOInputsAutoLogged input = inputs.get(index);
      io.updateInputs(input);
      Logger.processInputs(getName() + "/Camera" + index, input);
    }

    Logger.recordOutput(
        "vision/cameraPoses",
        ROBOT_TO_CAMERAS.stream()
            .map(transform -> new Pose3d(currentPose).transformBy(transform))
            .toArray(Pose3d[]::new));

    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allAcceptedRobotPoses = new ArrayList<>();
    List<Pose3d> allRejectedRobotPoses = new ArrayList<>();

    for (int index = 0; index < inputs.size(); index++) {
      VisionIOInputsAutoLogged input = inputs.get(index);
      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> acceptedRobotPoses = new ArrayList<>();
      List<Pose3d> rejectedRobotPoses = new ArrayList<>();

      for (int tagId : input.tagIds) {
        FieldConstants.TAG_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      PoseObservation bestObservation = null;
      int bestTagCount = -1;
      double bestAmbiguity = Double.POSITIVE_INFINITY;
      double bestAvgDist = Double.POSITIVE_INFINITY;

      for (PoseObservation observation : input.poseObservations) {
        boolean rejectPose = observation.tagCount() == 0
            || (observation.tagCount() == 1
                && (observation.ambiguity() > MAX_AMBIGUITY
                    || input.tagIds.length > 0
                        && !FieldConstants.Reef.REEF_TAGS.contains(input.tagIds[0])))
            || Math.abs(observation.pose().getZ()) > MAX_Z_ERROR
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > FieldConstants.TAG_LAYOUT.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > FieldConstants.TAG_LAYOUT.getFieldWidth();

        robotPoses.add(observation.pose());
        if (rejectPose) {
          rejectedRobotPoses.add(observation.pose());
          continue;
        }
        acceptedRobotPoses.add(observation.pose());

        boolean better = false;
        if (observation.tagCount() > bestTagCount) {
          better = true;
        } else if (observation.tagCount() == bestTagCount) {
          if (observation.ambiguity() < bestAmbiguity) {
            better = true;
          } else if (observation.ambiguity() == bestAmbiguity
              && observation.averageTagDistance() < bestAvgDist) {
            better = true;
          }
        }
        if (better) {
          bestObservation = observation;
          bestTagCount = observation.tagCount();
          bestAmbiguity = observation.ambiguity();
          bestAvgDist = observation.averageTagDistance();
        }
      }

      // Fuse at most one vision measurement per camera per cycle to reduce jitter
      if (bestObservation != null && !DriverStation.isDisabled() && linearSpeedMps >= MIN_LINEAR_SPEED_MPS) {
        Pose2d estPose = currentPose;
        Pose2d measPose = bestObservation.pose().toPose2d();
        double dx = measPose.getX() - estPose.getX();
        double dy = measPose.getY() - estPose.getY();
        double dist = Math.hypot(dx, dy);
        double dtheta = Math.abs(measPose.getRotation().minus(estPose.getRotation()).getRadians());

        // Reject large jumps relative to current estimate (helps in sim when tags pop
        // in/out)
        if (dist <= MAX_TRANSLATION_JUMP_METERS && dtheta <= MAX_ROTATION_JUMP_RAD) {
          double stdDevFactor = Math.pow(bestObservation.averageTagDistance(), 2) / bestObservation.tagCount();
          double clampedFactor = Math.max(1.0, stdDevFactor);
          double linearStdDev = LINEAR_STD_DEV_BASELINE * clampedFactor;
          double angularStdDev = ANGULAR_STD_DEV_BASELINE * clampedFactor;

          consumer.accept(
              measPose,
              bestObservation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      Logger.recordOutput(getName() + "/Camera" + index + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          getName() + "/Camera" + index + "/RobotPoses", robotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          getName() + "/Camera" + index + "/AcceptedRobotPoses",
          acceptedRobotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          getName() + "/Camera" + index + "/RejectedRobotPoses",
          rejectedRobotPoses.toArray(Pose3d[]::new));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allAcceptedRobotPoses.addAll(acceptedRobotPoses);
      allRejectedRobotPoses.addAll(rejectedRobotPoses);
    }

    // Save for next cycle speed estimate
    lastPose = currentPose;
    lastTimestampSec = nowSec;

    Logger.recordOutput(getName() + "/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(getName() + "/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        getName() + "/Summary/AcceptedRobotPoses",
        allAcceptedRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        getName() + "/Summary/RejectedRobotPoses",
        allRejectedRobotPoses.toArray(Pose3d[]::new));
  }

  private static final class NullVisionIO implements VisionIO {
  }
}
