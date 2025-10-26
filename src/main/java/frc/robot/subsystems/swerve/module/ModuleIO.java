package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.AutoLog;

/** Basic interface for swerve module hardware IO layers. */
public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveMotorConnected;
    public boolean turnMotorConnected;
    public boolean absoluteEncoderConnected;

    public double driveVelocityMetersPerSec;
    public double drivePositionMeters;
    public double drivePositionRad;
    public double driveAppliedVolts;
    public double driveStatorCurrentAmps;
    public double driveSupplyCurrentAmps;
    public double driveTorqueCurrentAmps;
    public double driveTargetVelocityRadPerSec;
    public double driveVelocityRadPerSec;

    public Rotation2d turnPosition = Rotation2d.kZero;
    public Rotation2d absoluteTurnPosition = Rotation2d.kZero;

    public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    public double turnStatorCurrentAmps;
    public double turnSupplyCurrentAmps;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setTurnPosition(Rotation2d position) {}

  default void setDriveVelocity(double velocityMetersPerSec) {}

  default void setDriveVelocity(double velocityMetersPerSec, double torqueCurrentAmps) {}

  default void reset() {}

  default void stop() {}

  default void setTurnBrake(boolean enabled) {}

  default void setDriveBrake(boolean enabled) {}

  default void setDrivePID(PIDGains gains) {}

  default void setSteerPID(PIDGains gains) {}

  default void setDriveCurrent(double currentAmps) {}
}
