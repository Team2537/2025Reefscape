package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorConstants;

/** Simulation manipulator implementation using simple DCMotorSim rollers. */
public final class ManipulatorIOSim implements ManipulatorIO {
  private final DCMotorSim leftRollerSim;
  private final DCMotorSim rightRollerSim;

  public ManipulatorIOSim() {
    this(ManipulatorConstants.ROLLER_GEARING, 0.01);
  }

  public ManipulatorIOSim(double rollerGearing, double rollerMoiKgM2) {
    leftRollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), rollerMoiKgM2, rollerGearing),
            DCMotor.getKrakenX60(1));
    rightRollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), rollerMoiKgM2, rollerGearing),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    leftRollerSim.update(0.02);
    rightRollerSim.update(0.02);

    inputs.leftRollerConnected = true;
    inputs.rightRollerConnected = true;
    inputs.leftRollerVelocityRadPerSec = leftRollerSim.getAngularVelocityRadPerSec();
    inputs.leftRollerAppliedVolts = leftRollerSim.getInputVoltage();
    inputs.leftRollerStatorCurrentAmps = leftRollerSim.getCurrentDrawAmps();
    inputs.rightRollerVelocityRadPerSec = rightRollerSim.getAngularVelocityRadPerSec();
    inputs.rightRollerAppliedVolts = rightRollerSim.getInputVoltage();
    inputs.rightRollerStatorCurrentAmps = rightRollerSim.getCurrentDrawAmps();
    inputs.coralDistanceMeters = 0.5; // placeholder
  }

  @Override
  public void setRollerVoltage(double volts) {
    leftRollerSim.setInputVoltage(volts);
    rightRollerSim.setInputVoltage(volts);
  }

  @Override
  public void setLeftRightRollerVoltages(double leftVolts, double rightVolts) {
    leftRollerSim.setInputVoltage(leftVolts);
    rightRollerSim.setInputVoltage(rightVolts);
  }

  @Override
  public void setRollerTorqueCurrent(double amps) {
    double volts = amps / 40.0 * 12.0;
    setRollerVoltage(volts);
  }

  @Override
  public void setLeftRightRollerTorqueCurrents(double leftAmps, double rightAmps) {
    setLeftRightRollerVoltages(leftAmps / 40.0 * 12.0, rightAmps / 40.0 * 12.0);
  }

  @Override
  public void stopRoller() {
    setRollerVoltage(0.0);
  }
}
