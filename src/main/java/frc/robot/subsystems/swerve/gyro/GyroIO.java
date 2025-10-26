package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for gyro hardware implementations. */
public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected;
    public Rotation2d yaw = new Rotation2d();
    public Rotation2d pitch = new Rotation2d();
    public Rotation2d roll = new Rotation2d();
    public double yawRateRadPerSec;
    public double pitchRateRadPerSec;
    public double rollRateRadPerSec;
    public Rotation3d fullRotation = new Rotation3d();
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void setYaw(Rotation2d heading) {}
}
