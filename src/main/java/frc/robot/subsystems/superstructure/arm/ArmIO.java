package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the arm subsystem. */
public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean leftMotorConnected;
    public boolean rightMotorConnected;
    public Rotation2d angle = new Rotation2d();
    public AngularVelocity angularVelocity;
    public Voltage appliedVolts;
    public Current leftStatorCurrent;
    public Current rightStatorCurrent;
  }

  default void updateInputs(ArmIOInputs inputs) {
  }

  default void setVoltage(Voltage volts) {
  }

  default void setTargetAngle(Rotation2d angle) {
  }

  default void setBrakeMode(boolean brake) {
  }

  default void stop() {
  }
}
