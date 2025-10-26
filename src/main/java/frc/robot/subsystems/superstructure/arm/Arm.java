package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotType;
import frc.robot.subsystems.superstructure.arm.ArmIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Arm subsystem wrapper around IO implementations. */
public final class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  public Arm() {
    this.io =
        switch (RobotType.MODE) {
          case REAL -> new ArmIOKraken();
          case SIMULATION -> new ArmIOSim();
          case REPLAY -> new ArmIO() {};
        };
    
    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Units.Volts.of(1.0),
            null,
            state -> Logger.recordOutput("arm/sysid", state.toString())),
        new SysIdRoutine.Mechanism(voltage -> io.setVoltage(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/AngleRad", inputs.angle.getRadians());
    Logger.recordOutput("Arm/VelocityRadPerSec", inputs.angularVelocityRadPerSec);
    Logger.recordOutput("Arm/AppliedVolts", inputs.appliedVolts);
  }

  private Rotation2d clampAngle(Rotation2d angle) {
    double min = ArmConstants.MIN_ANGLE.getRadians();
    double max = ArmConstants.MAX_ANGLE.getRadians();
    double clamped = Math.max(min, Math.min(max, angle.getRadians()));
    return Rotation2d.fromRadians(clamped);
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  public void setTargetAngle(Rotation2d angle) {
    io.setTargetAngle(clampAngle(angle));
  }

  public Command getMoveToAngleCommand(Supplier<Rotation2d> angleSupplier) {
    return Commands.runOnce(() -> setTargetAngle(angleSupplier.get()));
  }

  public Command getDynamicSysID(Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command getQuasistaticSysID(Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public ArmIO.ArmIOInputs getInputs() {
    return inputs;
  }

  public Rotation2d getAngle() {
    return inputs.angle;
  }
}
