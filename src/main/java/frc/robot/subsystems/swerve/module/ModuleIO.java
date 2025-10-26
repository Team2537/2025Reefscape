package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.AutoLog;

/** Basic interface for swerve module hardware IO layers. */
public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveMotorConnected;
    public boolean turnMotorConnected;
    public boolean absoluteEncoderConnected;

    public LinearVelocity driveVelocity;
    public Distance drivePosition;
    public edu.wpi.first.units.measure.Angle drivePositionRad;
    public Voltage driveAppliedVolts;
    public Current driveStatorCurrent;
    public Current driveSupplyCurrent;
    public Current driveTorqueCurrent;
    public AngularVelocity driveTargetVelocity;
    public AngularVelocity driveVelocityRadPerSec;

    public Rotation2d turnPosition = Rotation2d.kZero;
    public Rotation2d absoluteTurnPosition = Rotation2d.kZero;

    public AngularVelocity turnVelocity;
    public Voltage turnAppliedVolts;
    public Current turnStatorCurrent;
    public Current turnSupplyCurrent;
  }

  default void updateInputs(ModuleIOInputs inputs) {
  }

  default void setDriveVoltage(Voltage volts) {
  }

  default void setTurnVoltage(Voltage volts) {
  }

  default void setTurnPosition(Rotation2d position) {
  }

  default void setDriveVelocity(LinearVelocity velocity) {
  }

  default void setDriveVelocity(LinearVelocity velocity, Current torqueCurrent) {
  }

  default void reset() {
  }

  default void stop() {
  }

  default void setTurnBrake(boolean enabled) {
  }

  default void setDriveBrake(boolean enabled) {
  }

  default void setDrivePID(PIDGains gains) {
  }

  default void setSteerPID(PIDGains gains) {
  }

  default void setDriveCurrent(Current current) {
  }
}
