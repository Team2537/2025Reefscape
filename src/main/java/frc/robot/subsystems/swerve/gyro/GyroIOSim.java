package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import java.util.function.Supplier;

/** Simulated gyro that integrates chassis angular velocity. */
public final class GyroIOSim implements GyroIO {
  private final Supplier<ChassisSpeeds> speeds;

  public GyroIOSim(Supplier<ChassisSpeeds> speeds) {
    this.speeds = speeds;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double dt = Robot.getUpdateRateSec();
    double omega = speeds.get().omegaRadiansPerSecond;
    inputs.yaw = inputs.yaw.plus(Rotation2d.fromRadians(omega * dt));
    inputs.yawRateRadPerSec = omega;
    inputs.fullRotation = new Rotation3d(inputs.yaw);
    inputs.connected = true;
  }
}
