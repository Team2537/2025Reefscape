package lib.math.controllers.pathfollowing;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.swerve.Drivebase;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/** Path follower that additionally returns desired module forces for force-based control. */
public final class ModuleForcesPathFollower implements PathFollower {
  private final Drivebase drivebase;
  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;
  private final BiConsumer<ChassisSpeeds, List<Vector<N2>>> speedConsumer;
  private final Supplier<Pose2d> poseSupplier;

  public ModuleForcesPathFollower(
      Drivebase drivebase,
      PIDGains xPidGains,
      PIDGains yPidGains,
      PIDGains thetaPidGains,
      BiConsumer<ChassisSpeeds, List<Vector<N2>>> speedConsumer,
      Supplier<Pose2d> poseSupplier) {
    this.drivebase = drivebase;
    this.xPID = new PIDController(xPidGains.getKP(), xPidGains.getKI(), xPidGains.getKD());
    this.yPID = new PIDController(yPidGains.getKP(), yPidGains.getKI(), yPidGains.getKD());
    this.thetaPID =
        new PIDController(thetaPidGains.getKP(), thetaPidGains.getKI(), thetaPidGains.getKD());
    this.thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    this.speedConsumer = speedConsumer;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void accept(SwerveSample sample) {
    Pose2d pose = poseSupplier.get();

    double xOutput = xPID.calculate(pose.getX(), sample.getX());
    double yOutput = yPID.calculate(pose.getY(), sample.getY());
    double thetaOutput = thetaPID.calculate(pose.getRotation().getRadians(), sample.getHeading());

    Logger.recordOutput("drivebase/auto/xError", xPID.getPositionError());
    Logger.recordOutput("drivebase/auto/yError", yPID.getPositionError());
    Logger.recordOutput("drivebase/auto/thetaError", thetaPID.getPositionError());

    Logger.recordOutput("drivebase/auto/samplePose", Pose2d.struct, sample.getPose());
    Logger.recordOutput("drivebase/auto/pose", Pose2d.struct, pose);

    List<Double> moduleForcesX = sample.getModuleForcesX();
    List<Double> moduleForcesY = sample.getModuleForcesY();
    int moduleCount = Math.min(moduleForcesX.size(), moduleForcesY.size());
    List<Vector<N2>> moduleForces = new ArrayList<>(moduleCount);
    for (int i = 0; i < moduleCount; i++) {
      moduleForces.add(VecBuilder.fill(moduleForcesX.get(i), moduleForcesY.get(i)));
    }

    speedConsumer.accept(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput + sample.getVx(),
            yOutput + sample.getVy(),
            thetaOutput + sample.getOmega(),
            pose.getRotation()),
        moduleForces);
  }
}
