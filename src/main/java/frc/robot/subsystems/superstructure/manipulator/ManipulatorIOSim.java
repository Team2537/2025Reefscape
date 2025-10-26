package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorConstants;

/** Simulation manipulator implementation using simple DCMotorSim rollers. */
public final class ManipulatorIOSim implements ManipulatorIO {
  private final DCMotorSim leftRollerSim;
  private final DCMotorSim rightRollerSim;

  public ManipulatorIOSim() {
    this(ManipulatorConstants.ROLLER_GEARING, ManipulatorConstants.SIM_ROLLER_MOI.in(Units.KilogramSquareMeters));
  }

  public ManipulatorIOSim(double rollerGearing, double rollerMoiKgM2) {
    leftRollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), rollerMoiKgM2, rollerGearing),
        DCMotor.getKrakenX60(1));
    rightRollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), rollerMoiKgM2, rollerGearing),
        DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    leftRollerSim.update(0.02);
    rightRollerSim.update(0.02);

    inputs.leftRollerConnected = true;
    inputs.rightRollerConnected = true;
    inputs.leftRollerVelocity = Units.RadiansPerSecond.of(leftRollerSim.getAngularVelocityRadPerSec());
    inputs.leftRollerAppliedVolts = Units.Volts.of(leftRollerSim.getInputVoltage());
    inputs.leftRollerStatorCurrent = Units.Amps.of(leftRollerSim.getCurrentDrawAmps());
    inputs.leftRollerTorqueCurrent = Units.Amps.zero();
    inputs.rightRollerVelocity = Units.RadiansPerSecond.of(rightRollerSim.getAngularVelocityRadPerSec());
    inputs.rightRollerAppliedVolts = Units.Volts.of(rightRollerSim.getInputVoltage());
    inputs.rightRollerStatorCurrent = Units.Amps.of(rightRollerSim.getCurrentDrawAmps());
    inputs.rightRollerTorqueCurrent = Units.Amps.zero();
    inputs.coralDistance = Units.Meters.of(0.5); // placeholder
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    leftRollerSim.setInputVoltage(volts.in(Units.Volts));
    rightRollerSim.setInputVoltage(volts.in(Units.Volts));
  }

  @Override
  public void setLeftRightRollerVoltages(Voltage leftVolts, Voltage rightVolts) {
    leftRollerSim.setInputVoltage(leftVolts.in(Units.Volts));
    rightRollerSim.setInputVoltage(rightVolts.in(Units.Volts));
  }

  @Override
  public void setRollerTorqueCurrent(Current amps) {
    double volts = amps.in(Units.Amps) / 40.0 * 12.0;
    setRollerVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setLeftRightRollerTorqueCurrents(Current leftAmps, Current rightAmps) {
    setLeftRightRollerVoltages(
        Units.Volts.of(leftAmps.in(Units.Amps) / 40.0 * 12.0),
        Units.Volts.of(rightAmps.in(Units.Amps) / 40.0 * 12.0));
  }

  @Override
  public void stopRoller() {
    setRollerVoltage(Units.Volts.zero());
  }
}
