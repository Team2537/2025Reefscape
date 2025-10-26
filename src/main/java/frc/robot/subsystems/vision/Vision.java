package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lib.math.geometry.FieldConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

/** Vision subsystem responsible for processing pose estimates from PhotonVision. */
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
  private static final double LINEAR_STD_DEV_BASELINE = 0.02;
  private static final double ANGULAR_STD_DEV_BASELINE = 0.06;

  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final List<VisionIO> ios;
  private final List<VisionIOInputsAutoLogged> inputs;

  public Vision(VisionConsumer consumer) {
    super("vision");
    this.consumer = consumer;
    this.robotPoseSupplier = () -> Robot.getDrivebase().getPose();
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
    for (int index = 0; index < ios.size(); index++) {
      VisionIO io = ios.get(index);
      VisionIOInputsAutoLogged input = inputs.get(index);
      io.updateInputs(input);
      Logger.processInputs(getName() + "/Camera" + index, input);
    }

    Logger.recordOutput(
        "vision/cameraPoses",
        ROBOT_TO_CAMERAS.stream()
            .map(transform -> new Pose3d(robotPoseSupplier.get()).transformBy(transform))
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

      for (PoseObservation observation : input.poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0
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

        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2) / observation.tagCount();
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
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

    Logger.recordOutput(getName() + "/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(getName() + "/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        getName() + "/Summary/AcceptedRobotPoses",
        allAcceptedRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        getName() + "/Summary/RejectedRobotPoses",
        allRejectedRobotPoses.toArray(Pose3d[]::new));
  }

  private static final class NullVisionIO implements VisionIO {}
}
