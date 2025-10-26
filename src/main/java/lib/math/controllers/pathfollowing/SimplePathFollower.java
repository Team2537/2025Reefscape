package lib.math.controllers.pathfollowing;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Basic holonomic path follower that converts pose error into chassis speeds.
 */
public final class SimplePathFollower implements PathFollower {
  private final Drive drive;
  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;
  private final Consumer<ChassisSpeeds> speedConsumer;
  private final Supplier<Pose2d> poseSupplier;

  public SimplePathFollower(
      Drive drive,
      PIDGains xPidGains,
      PIDGains yPidGains,
      PIDGains thetaPidGains,
      Consumer<ChassisSpeeds> speedConsumer,
      Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.xPID = new PIDController(xPidGains.getKP(), xPidGains.getKI(), xPidGains.getKD());
    this.yPID = new PIDController(yPidGains.getKP(), yPidGains.getKI(), yPidGains.getKD());
    this.thetaPID = new PIDController(thetaPidGains.getKP(), thetaPidGains.getKI(), thetaPidGains.getKD());
    this.thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    this.speedConsumer = speedConsumer;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void accept(SwerveSample sample) {
    Pose2d pose = poseSupplier.get();

    double xOutput = xPID.calculate(pose.getX(), sample.x);
    double yOutput = yPID.calculate(pose.getY(), sample.y);
    double thetaOutput = thetaPID.calculate(pose.getRotation().getRadians(), sample.heading);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xOutput + sample.vx,
        yOutput + sample.vy,
        thetaOutput + sample.omega,
        pose.getRotation());

    Logger.recordOutput("drivebase/auto/xError", sample.x - pose.getX());
    Logger.recordOutput("drivebase/auto/yError", sample.y - pose.getY());
    Logger.recordOutput("drivebase/auto/thetaError", sample.heading - pose.getRotation().getRadians());

    Logger.recordOutput("drivebase/auto/samplePose", Pose2d.struct, sample.getPose());
    Logger.recordOutput("drivebase/auto/pose", Pose2d.struct, pose);

    if (Math.abs(speeds.vxMetersPerSecond) < Units.inchesToMeters(1.0)
        && Math.abs(speeds.vyMetersPerSecond) < Units.inchesToMeters(1.0)
        && Math.abs(speeds.omegaRadiansPerSecond) < Units.degreesToRadians(1.0)) {
      ChassisSpeeds zeroSpeeds = new ChassisSpeeds();
      speedConsumer.accept(zeroSpeeds);
      Logger.recordOutput("drivebase/auto/speeds", ChassisSpeeds.struct, zeroSpeeds);
    } else {
      speedConsumer.accept(speeds);
      Logger.recordOutput("drivebase/auto/speeds", ChassisSpeeds.struct, speeds);
    }
  }
}
