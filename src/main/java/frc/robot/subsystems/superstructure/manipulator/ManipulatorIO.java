package frc.robot.subsystems.superstructure.manipulator;

import org.littletonrobotics.junction.AutoLog;

/** IO layer for the manipulator rollers. */
public interface ManipulatorIO {
  @AutoLog
  class ManipulatorIOInputs {
    public boolean leftRollerConnected;
    public boolean rightRollerConnected;

    public double leftRollerVelocityRadPerSec;
    public double leftRollerAppliedVolts;
    public double leftRollerStatorCurrentAmps;
    public double leftRollerTorqueCurrentAmps;

    public double rightRollerVelocityRadPerSec;
    public double rightRollerAppliedVolts;
    public double rightRollerStatorCurrentAmps;
    public double rightRollerTorqueCurrentAmps;

    public double coralDistanceMeters;
  }

  default void updateInputs(ManipulatorIOInputs inputs) {}

  default void setRollerVoltage(double volts) {}

  default void setLeftRightRollerVoltages(double leftVolts, double rightVolts) {}

  default void setRollerTorqueCurrent(double amps) {}

  default void setLeftRightRollerTorqueCurrents(double leftAmps, double rightAmps) {}

  default void stopRoller() {}
}
