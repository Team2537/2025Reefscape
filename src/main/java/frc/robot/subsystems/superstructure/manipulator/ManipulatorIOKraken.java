package frc.robot.subsystems.superstructure.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StatusSignal;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ManipulatorConstants;

/** Real manipulator implementation using Talon FX rollers. */
public final class ManipulatorIOKraken implements ManipulatorIO {
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC leftTorqueRequest = new TorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC rightTorqueRequest = new TorqueCurrentFOC(0.0);

  private double leftTorqueCommandAmps = 0.0;
  private double rightTorqueCommandAmps = 0.0;

  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftVoltage;
  private final StatusSignal<Double> leftCurrent;

  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightVoltage;
  private final StatusSignal<Double> rightCurrent;

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
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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
    inputs.leftRollerVelocityRadPerSec = leftVelocity.getValue();
    inputs.leftRollerAppliedVolts = leftVoltage.getValue();
    inputs.leftRollerStatorCurrentAmps = leftCurrent.getValue();
    inputs.leftRollerTorqueCurrentAmps = leftTorqueCommandAmps;
    inputs.rightRollerVelocityRadPerSec = rightVelocity.getValue();
    inputs.rightRollerAppliedVolts = rightVoltage.getValue();
    inputs.rightRollerStatorCurrentAmps = rightCurrent.getValue();
    inputs.rightRollerTorqueCurrentAmps = rightTorqueCommandAmps;
    inputs.coralDistanceMeters = Double.NaN;
  }

  @Override
  public void setRollerVoltage(double volts) {
    leftRollerMotor.setControl(leftVoltageRequest.withOutput(volts));
    rightRollerMotor.setControl(rightVoltageRequest.withOutput(volts));
  }

  @Override
  public void setLeftRightRollerVoltages(double leftVolts, double rightVolts) {
    leftRollerMotor.setControl(leftVoltageRequest.withOutput(leftVolts));
    rightRollerMotor.setControl(rightVoltageRequest.withOutput(rightVolts));
  }

  @Override
  public void setRollerTorqueCurrent(double amps) {
    leftTorqueCommandAmps = amps;
    rightTorqueCommandAmps = amps;
    leftRollerMotor.setControl(leftTorqueRequest.withOutput(Units.Amps.of(amps)));
    rightRollerMotor.setControl(rightTorqueRequest.withOutput(Units.Amps.of(amps)));
  }

  @Override
  public void setLeftRightRollerTorqueCurrents(double leftAmps, double rightAmps) {
    leftTorqueCommandAmps = leftAmps;
    rightTorqueCommandAmps = rightAmps;
    leftRollerMotor.setControl(leftTorqueRequest.withOutput(Units.Amps.of(leftAmps)));
    rightRollerMotor.setControl(rightTorqueRequest.withOutput(Units.Amps.of(rightAmps)));
  }

  @Override
  public void stopRoller() {
    setRollerVoltage(0.0);
  }
}
