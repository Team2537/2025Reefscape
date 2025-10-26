package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** IO layer for the manipulator rollers. */
public interface ManipulatorIO {
  @AutoLog
  class ManipulatorIOInputs {
    public boolean leftRollerConnected;
    public boolean rightRollerConnected;

    public AngularVelocity leftRollerVelocity;
    public Voltage leftRollerAppliedVolts;
    public Current leftRollerStatorCurrent;
    public Current leftRollerTorqueCurrent;

    public AngularVelocity rightRollerVelocity;
    public Voltage rightRollerAppliedVolts;
    public Current rightRollerStatorCurrent;
    public Current rightRollerTorqueCurrent;

    public Distance coralDistance;
  }

  default void updateInputs(ManipulatorIOInputs inputs) {
  }

  default void setRollerVoltage(Voltage volts) {
  }

  default void setLeftRightRollerVoltages(Voltage leftVolts, Voltage rightVolts) {
  }

  default void setRollerTorqueCurrent(Current amps) {
  }

  default void setLeftRightRollerTorqueCurrents(Current leftAmps, Current rightAmps) {
  }

  default void stopRoller() {
  }
}
