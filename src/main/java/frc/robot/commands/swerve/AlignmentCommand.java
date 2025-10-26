package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetTransform;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import lib.math.geometry.FieldConstants;
import lib.math.geometry.GeometryUtil;
import lib.math.geometry.FieldFlipUtil;
import org.littletonrobotics.junction.Logger;

/** Command for aligning the drivebase to various field references. */
public final class AlignmentCommand extends Command {
  private static final PathConstraints ALIGN_LIMITS = new PathConstraints(
      edu.wpi.first.units.Units.MetersPerSecond.of(1.5),
      edu.wpi.first.units.Units.MetersPerSecondPerSecond.of(8.0),
      edu.wpi.first.units.Units.DegreesPerSecond.of(540.0),
      edu.wpi.first.units.Units.DegreesPerSecondPerSecond.of(720.0));

  private final Drive drive;
  private final AlignmentState alignmentState;
  private final Supplier<Pose2d> poseSupplier;
  private final PIDController xPid;
  private final PIDController yPid;
  private final PIDController rotPid;
  private final Supplier<AlignmentState.State> endStateSupplier;

  private Pose2d targetPose;
  private AlignmentState.State endState = AlignmentState.State.DRIVING;
  private PathConstraints previousLimits;

  public AlignmentCommand(
      Drive drive,
      AlignmentState alignmentState,
      Supplier<Pose2d> poseSupplier,
      PIDController translationPid,
      PIDController rotationPid,
      Supplier<AlignmentState.State> endStateSupplier) {
    this.drive = drive;
    this.alignmentState = alignmentState;
    this.poseSupplier = poseSupplier;
    this.xPid = translationPid;
    this.yPid = new PIDController(translationPid.getP(), translationPid.getI(), translationPid.getD());
    this.rotPid = rotationPid;
    this.endStateSupplier = endStateSupplier;

    this.xPid.setTolerance(Units.inchesToMeters(0.1));
    this.yPid.setTolerance(Units.inchesToMeters(0.1));
    this.rotPid.setTolerance(Units.degreesToRadians(2.0));
    this.rotPid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    targetPose = null;
    previousLimits = alignmentState.getLimits();
    alignmentState.setState(AlignmentState.State.ALIGNING);
    alignmentState.setLimits(ALIGN_LIMITS);

    xPid.reset();
    yPid.reset();
    rotPid.reset();

    endState = endStateSupplier.get();
    targetPose = poseSupplier.get();
    logTarget();
  }

  private void logTarget() {
    if (targetPose != null) {
      Logger.recordOutput("commands/" + getName() + "/TargetPose", Pose2d.struct, targetPose);
    }
  }

  @Override
  public void execute() {
    targetPose = poseSupplier.get();
    logTarget();
    if (targetPose == null) {
      return;
    }

    Pose2d robotPose = drive.getPose();
    double xOutput = xPid.calculate(robotPose.getX(), targetPose.getX());
    double yOutput = yPid.calculate(robotPose.getY(), targetPose.getY());
    double thetaOutput = rotPid.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xOutput, yOutput, thetaOutput, robotPose.getRotation());
    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return targetPose == null || (xPid.atSetpoint() && yPid.atSetpoint() && rotPid.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    alignmentState.setState(endState);
    alignmentState.setLimits(previousLimits != null ? previousLimits : DriveConstants.DEFAULT_LIMITS);
  }

  public static Command buttonBoardAlign(
      Drive drive,
      AlignmentState alignmentState,
      Vision vision,
      FieldConstants.Reef.ReefFace face,
      Trigger isL4,
      Trigger leftSupplier,
      Trigger rightSupplier,
      Trigger centerSupplier) {
    return new AlignmentCommand(
        drive,
        alignmentState,
        () -> computeButtonBoardTarget(drive, face, leftSupplier, rightSupplier, centerSupplier),
        new PIDController(10.0, 0.0, 0.01),
        new PIDController(5.0, 0.0, 0.0),
        () -> (rightSupplier.getAsBoolean() || leftSupplier.getAsBoolean()) && !centerSupplier.getAsBoolean()
            ? AlignmentState.State.ALIGNED_CORAL
            : AlignmentState.State.ALIGNED_ALGAE)
        .withName("ButtonBoardAlign");
  }

  private static Pose2d computeButtonBoardTarget(
      Drive drive,
      FieldConstants.Reef.ReefFace face,
      Trigger leftSupplier,
      Trigger rightSupplier,
      Trigger centerSupplier) {
    Pose2d target = FieldConstants.Reef.FLOOR_ALIGNMENT_POSES.get(face.ordinal());
    if (AutoBuilder.shouldFlip()) {
      target = FieldFlipUtil.flip(target);
    }

    double offsetY = 0.0;
    double manipulatorOffset = Units.inchesToMeters(6.0);
    if (leftSupplier.getAsBoolean() && !rightSupplier.getAsBoolean() && !centerSupplier.getAsBoolean()) {
      offsetY = (FieldConstants.Reef.SIDE_OFFSET_METERS / 2.0) - manipulatorOffset;
    } else if (rightSupplier.getAsBoolean() && !leftSupplier.getAsBoolean() && !centerSupplier.getAsBoolean()) {
      offsetY = -((FieldConstants.Reef.SIDE_OFFSET_METERS / 2.0) + manipulatorOffset);
    }

    Pose2d nudged = GeometryUtil.nudge(target, 0.0, offsetY);
    return nudged.getTranslation().getDistance(drive.getPose().getTranslation()) <= 1.5 ? nudged : null;
  }

  public static Command sourceAlignment(Drive drive, AlignmentState alignmentState) {
    return new AlignmentCommand(
        drive,
        alignmentState,
        () -> computeSourceTarget(drive),
        new PIDController(5.0, 0.0, 0.05),
        new PIDController(5.0, 0.0, 0.0),
        () -> AlignmentState.State.ALIGNED_SOURCE)
        .withTimeout(3.0)
        .withName("SourceAlignmentCommand");
  }

  private static Pose2d computeSourceTarget(Drive drive) {
    Pose2d current = drive.getPose();
    if (AutoBuilder.shouldFlip()) {
      current = FieldFlipUtil.flip(current);
    }

    final Pose2d finalCurrent = current;
    List<Pose2d> targets = List.of(FieldConstants.Source.BLUE_TOP_SOURCE_CENTER,
        FieldConstants.Source.BLUE_BOTTOM_SOURCE_CENTER);
    Pose2d nearest = targets.stream()
        .min(Comparator.comparingDouble(t -> t.getTranslation().getDistance(finalCurrent.getTranslation())))
        .orElse(null);
    if (nearest == null) {
      return null;
    }
    if (AutoBuilder.shouldFlip()) {
      nearest = FieldFlipUtil.flip(nearest);
    }
    return nearest.getTranslation().getDistance(drive.getPose().getTranslation()) <= 1.0 ? nearest : null;
  }

  public static Command tagRelativeAlign(
      Drive drive,
      AlignmentState alignmentState,
      Vision vision,
      double standoffMeters,
      double lateralOffsetMeters) {
    final int[] phase = { 0 };
    return new AlignmentCommand(
        drive,
        alignmentState,
        () -> computeTagRelativeTarget(drive, vision, standoffMeters, lateralOffsetMeters, phase),
        new PIDController(3.5, 0.01, 0.02),
        new PIDController(3.0, 0.0, 0.03),
        () -> AlignmentState.State.ALIGNED_CORAL)
        .withTimeout(3.0)
        .beforeStarting(() -> phase[0] = 0)
        .withName("TagRelativeAlign");
  }

  private static Pose2d computeTagRelativeTarget(
      Drive drive,
      Vision vision,
      double standoffMeters,
      double lateralOffsetMeters,
      int[] phase) {
    List<TargetTransform> transforms = vision.getTargetTransforms();
    transforms = transforms.stream()
        .filter(t -> FieldConstants.Reef.REEF_TAGS.contains(t.fiducialId()))
        .toList();
    if (transforms.isEmpty()) {
      return null;
    }

    TargetTransform best = transforms.stream().min(Comparator.comparingDouble(TargetTransform::distance)).orElse(null);
    if (best == null) {
      return null;
    }

    Transform3d robotToTag = vision.getRobotToCameras().get(best.cameraIndex()).plus(best.cameraToTarget());
    double currX = robotToTag.getTranslation().getX();
    double currY = robotToTag.getTranslation().getY();
    double yaw = robotToTag.getRotation().getZ();

    double bearing = Math.atan2(currY, currX);
    double distance = Math.hypot(currX, currY);
    double radialDelta = distance - standoffMeters;
    double lateralDelta = currY - lateralOffsetMeters;

    double yawTol = Math.toRadians(3.0);
    double latTol = 0.03;

    if (phase[0] == 0 && Math.abs(yaw) < yawTol)
      phase[0] = 1;
    if (phase[0] == 1 && Math.abs(lateralDelta) < latTol)
      phase[0] = 2;

    double deltaX;
    double deltaY;
    double desiredYaw;
    if (phase[0] == 0) {
      deltaX = 0.0;
      deltaY = 0.0;
      desiredYaw = yaw;
    } else if (phase[0] == 1) {
      deltaX = 0.0;
      deltaY = lateralDelta;
      desiredYaw = yaw;
    } else {
      deltaX = radialDelta * Math.cos(bearing);
      deltaY = radialDelta * Math.sin(bearing);
      desiredYaw = yaw;
    }

    Pose2d robotPose = drive.getPose();
    Rotation2d desiredHeading = robotPose.getRotation().plus(Rotation2d.fromRadians(desiredYaw));

    Pose2d fieldTarget = new Pose2d(
        robotPose.getX() + deltaX * robotPose.getRotation().getCos() - deltaY * robotPose.getRotation().getSin(),
        robotPose.getY() + deltaX * robotPose.getRotation().getSin() + deltaY * robotPose.getRotation().getCos(),
        desiredHeading);

    Logger.recordOutput("commands/TagRelativeAlign/phase", phase[0]);
    Logger.recordOutput("commands/TagRelativeAlign/desiredPose", Pose2d.struct, fieldTarget);
    return fieldTarget;
  }
}
