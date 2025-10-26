package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the arm subsystem. */
public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean leftMotorConnected;
    public boolean rightMotorConnected;
    public Rotation2d angle = new Rotation2d();
    public double angularVelocityRadPerSec;
    public double appliedVolts;
    public double leftStatorCurrentAmps;
    public double rightStatorCurrentAmps;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setTargetAngle(Rotation2d angle) {}

  default void setBrakeMode(boolean brake) {}

  default void stop() {}
}
