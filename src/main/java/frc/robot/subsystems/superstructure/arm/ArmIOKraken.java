package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ArmConstants;

/** Arm IO implementation for dual Talon FX motors. */
public final class ArmIOKraken implements ArmIO {
  private final TalonFX left = new TalonFX(ArmConstants.LEFT_MOTOR_ID);
  private final TalonFX right = new TalonFX(ArmConstants.RIGHT_MOTOR_ID);

  private final StatusSignal<Double> leftPosition = left.getPosition().clone();
  private final StatusSignal<Double> leftVelocity = left.getVelocity().clone();
  private final StatusSignal<Double> leftVoltage = left.getMotorVoltage().clone();
  private final StatusSignal<Double> leftCurrent = left.getStatorCurrent().clone();
  private final StatusSignal<Double> rightCurrent = right.getStatorCurrent().clone();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  public ArmIOKraken() {
    double mechanismRatio = ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO;

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted =
        ArmConstants.LEFT_INVERTED
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Feedback.SensorToMechanismRatio = mechanismRatio;
    leftConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.Slot0.kP = ArmConstants.KP;
    leftConfig.Slot0.kI = ArmConstants.KI;
    leftConfig.Slot0.kD = ArmConstants.KD;
    leftConfig.Slot0.kS = ArmConstants.KS;
    leftConfig.Slot0.kG = ArmConstants.KG;
    leftConfig.Slot0.kV = ArmConstants.KV;
    leftConfig.Slot0.kA = ArmConstants.KA;
    leftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    leftConfig.MotionMagic.withMotionMagicCruiseVelocity(Units.RotationsPerSecond.of(0.5));
    leftConfig.MotionMagic.withMotionMagicAcceleration(Units.RotationsPerSecondPerSecond.of(1.0));
    left.getConfigurator().apply(leftConfig);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted =
        ArmConstants.RIGHT_INVERTED
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.Feedback.SensorToMechanismRatio = mechanismRatio;
    rightConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.Slot0.kP = ArmConstants.KP;
    rightConfig.Slot0.kI = ArmConstants.KI;
    rightConfig.Slot0.kD = ArmConstants.KD;
    rightConfig.Slot0.kS = ArmConstants.KS;
    rightConfig.Slot0.kG = ArmConstants.KG;
    rightConfig.Slot0.kV = ArmConstants.KV;
    rightConfig.Slot0.kA = ArmConstants.KA;
    rightConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    rightConfig.MotionMagic.withMotionMagicCruiseVelocity(Units.RotationsPerSecond.of(0.5));
    rightConfig.MotionMagic.withMotionMagicAcceleration(Units.RotationsPerSecondPerSecond.of(1.0));
    right.getConfigurator().apply(rightConfig);

    double startingRot = Rotation2d.fromDegrees(ArmConstants.STARTING_ANGLE.in(Units.Degrees)).getRotations();
    left.setPosition(startingRot);
    right.setPosition(startingRot);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftVoltage, leftCurrent, rightCurrent);
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
    inputs.angle = Rotation2d.fromRotations(leftPosition.getValue());
    inputs.angularVelocityRadPerSec = leftVelocity.getValue();
    inputs.appliedVolts = leftVoltage.getValue();
    inputs.leftStatorCurrentAmps = leftCurrent.getValue();
    inputs.rightStatorCurrentAmps = rightCurrent.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    left.setControl(voltageRequest.withOutput(volts));
    right.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    double rotations = angle.getRotations();
    left.setControl(motionMagicRequest.withPosition(rotations));
    right.setControl(motionMagicRequest.withPosition(rotations));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    left.getConfigurator().refresh(leftConfig);
    leftConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    left.getConfigurator().apply(leftConfig);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    right.getConfigurator().refresh(rightConfig);
    rightConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    right.getConfigurator().apply(rightConfig);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
