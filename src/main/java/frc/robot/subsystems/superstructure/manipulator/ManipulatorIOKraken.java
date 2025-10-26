package frc.robot.subsystems.superstructure.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ManipulatorConstants;

/** Real manipulator implementation using Talon FX rollers. */
public final class ManipulatorIOKraken implements ManipulatorIO {
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC leftTorqueRequest = new TorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC rightTorqueRequest = new TorqueCurrentFOC(0.0);

  private Current leftTorqueCommand = Units.Amps.zero();
  private Current rightTorqueCommand = Units.Amps.zero();

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftVoltage;
  private final StatusSignal<Current> leftCurrent;

  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightVoltage;
  private final StatusSignal<Current> rightCurrent;

  public ManipulatorIOKraken() {
    leftRollerMotor = configureMotor(
        ManipulatorConstants.LEFT_ROLLER_MOTOR_ID,
        ManipulatorConstants.LEFT_ROLLER_INVERTED,
        ManipulatorConstants.ROLLER_GEARING);
    rightRollerMotor = configureMotor(
        ManipulatorConstants.RIGHT_ROLLER_MOTOR_ID,
        ManipulatorConstants.RIGHT_ROLLER_INVERTED,
        ManipulatorConstants.ROLLER_GEARING);

    leftVelocity = leftRollerMotor.getVelocity().clone();
    leftVoltage = leftRollerMotor.getMotorVoltage().clone();
    leftCurrent = leftRollerMotor.getStatorCurrent().clone();

    rightVelocity = rightRollerMotor.getVelocity().clone();
    rightVoltage = rightRollerMotor.getMotorVoltage().clone();
    rightCurrent = rightRollerMotor.getStatorCurrent().clone();
  }

  private TalonFX configureMotor(int id, boolean inverted, double gearing) {
    TalonFX motor = new TalonFX(id);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = gearing;
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);
    return motor;
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVelocity, leftVoltage, leftCurrent, rightVelocity, rightVoltage, rightCurrent);

    inputs.leftRollerConnected = true;
    inputs.rightRollerConnected = true;
    inputs.leftRollerVelocity = leftVelocity.getValue();
    inputs.leftRollerAppliedVolts = leftVoltage.getValue();
    inputs.leftRollerStatorCurrent = leftCurrent.getValue();
    inputs.leftRollerTorqueCurrent = leftTorqueCommand;
    inputs.rightRollerVelocity = rightVelocity.getValue();
    inputs.rightRollerAppliedVolts = rightVoltage.getValue();
    inputs.rightRollerStatorCurrent = rightCurrent.getValue();
    inputs.rightRollerTorqueCurrent = rightTorqueCommand;
    inputs.coralDistance = Units.Meters.of(Double.NaN);
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    leftRollerMotor.setControl(leftVoltageRequest.withOutput(volts.in(Units.Volts)));
    rightRollerMotor.setControl(rightVoltageRequest.withOutput(volts.in(Units.Volts)));
  }

  @Override
  public void setLeftRightRollerVoltages(Voltage leftVolts, Voltage rightVolts) {
    leftRollerMotor.setControl(leftVoltageRequest.withOutput(leftVolts.in(Units.Volts)));
    rightRollerMotor.setControl(rightVoltageRequest.withOutput(rightVolts.in(Units.Volts)));
  }

  @Override
  public void setRollerTorqueCurrent(Current amps) {
    leftTorqueCommand = amps;
    rightTorqueCommand = amps;
    leftRollerMotor.setControl(leftTorqueRequest.withOutput(amps.in(Units.Amps)));
    rightRollerMotor.setControl(rightTorqueRequest.withOutput(amps.in(Units.Amps)));
  }

  @Override
  public void setLeftRightRollerTorqueCurrents(Current leftAmps, Current rightAmps) {
    leftTorqueCommand = leftAmps;
    rightTorqueCommand = rightAmps;
    leftRollerMotor.setControl(leftTorqueRequest.withOutput(leftAmps.in(Units.Amps)));
    rightRollerMotor.setControl(rightTorqueRequest.withOutput(rightAmps.in(Units.Amps)));
  }

  @Override
  public void stopRoller() {
    setRollerVoltage(Units.Volts.zero());
  }
}
