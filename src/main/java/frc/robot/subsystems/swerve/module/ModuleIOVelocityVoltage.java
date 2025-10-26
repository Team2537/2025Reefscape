package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;

/** Module IO implementation using Talon FX for drive and SparkMax for steer. */
public final class ModuleIOVelocityVoltage implements ModuleIO {
  private final TalonFX driveMotor;
  private final SparkMax turnMotor;
  private final SparkClosedLoopController turnPidController;
  private final CANcoder absoluteEncoder;
  private final double wheelRadiusMeters;

  private final VoltageOut openLoopDriveRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC openLoopTorqueRequest = new TorqueCurrentFOC(0.0);
  private final VelocityVoltage closedLoopDriveRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveSupplyVolts;
  private final StatusSignal<Voltage> driveMotorVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  private final StatusSignal<Angle> absoluteTurnPosition;

  private final SparkMaxConfig turnBrakeConfig;
  private final SparkMaxConfig turnCoastConfig;
  private final FeedforwardGains turnFF;

  public ModuleIOVelocityVoltage(
      int driveID,
      double driveGearing,
      boolean driveInverted,
      FeedforwardGains driveFF,
      PIDGains drivePID,
      int turnID,
      double turnGearing,
      boolean turnInverted,
      FeedforwardGains turnFF,
      PIDGains turnPID,
      int encoderID,
      Rotation2d encoderOffset,
      Distance wheelRadius) {
    this.wheelRadiusMeters = wheelRadius.in(Units.Meter);
    this.turnFF = turnFF;

    driveMotor = new TalonFX(driveID);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback.SensorToMechanismRatio = driveGearing;
    driveConfig.Slot0.kP = drivePID.getKP();
    driveConfig.Slot0.kI = drivePID.getKI();
    driveConfig.Slot0.kD = drivePID.getKD();
    driveConfig.Slot0.kV = driveFF.getKV();
    driveConfig.Slot0.kA = driveFF.getKA();
    driveConfig.Slot0.kS = driveFF.getKS();
    driveConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.TorqueCurrent.withPeakForwardTorqueCurrent(120.0);
    driveConfig.TorqueCurrent.withPeakReverseTorqueCurrent(-120.0);
    driveConfig.MotorOutput.Inverted = driveInverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig);

    drivePosition = driveMotor.getPosition().clone();
    driveVelocity = driveMotor.getVelocity().clone();
    driveSupplyVolts = driveMotor.getSupplyVoltage().clone();
    driveMotorVolts = driveMotor.getMotorVoltage().clone();
    driveStatorCurrent = driveMotor.getStatorCurrent().clone();
    driveSupplyCurrent = driveMotor.getSupplyCurrent().clone();
    driveTorqueCurrent = driveMotor.getTorqueCurrent().clone();

    turnBrakeConfig = createTurnConfig(
        turnPID,
        turnGearing,
        turnInverted,
        SparkBaseConfig.IdleMode.kBrake);
    turnCoastConfig = createTurnConfig(
        turnPID,
        turnGearing,
        turnInverted,
        SparkBaseConfig.IdleMode.kCoast);

    turnMotor = new SparkMax(turnID, SparkLowLevel.MotorType.kBrushless);
    turnMotor.configure(
        turnBrakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    turnPidController = turnMotor.getClosedLoopController();

    absoluteEncoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    absoluteEncoder.getConfigurator().apply(encoderConfig);
    absoluteTurnPosition = absoluteEncoder.getAbsolutePosition().clone();

    turnMotor.getEncoder().setPosition(absoluteTurnPosition.getValueAsDouble());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveSupplyVolts,
        driveMotorVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        driveTorqueCurrent)
        .isOK();

    inputs.turnMotorConnected = true;
    inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(absoluteTurnPosition).isOK();

    inputs.driveVelocityRadPerSec = driveVelocity.getValue();
    inputs.driveVelocity = Units.MetersPerSecond.of(driveVelocity.getValueAsDouble() * wheelRadiusMeters);
    inputs.drivePositionRad = Units.Radians.of(drivePosition.getValueAsDouble());
    inputs.drivePosition = Units.Meters.of(drivePosition.getValueAsDouble() * wheelRadiusMeters);
    inputs.driveAppliedVolts = driveMotorVolts.getValue();
    inputs.driveStatorCurrent = driveStatorCurrent.getValue();
    inputs.driveSupplyCurrent = driveSupplyCurrent.getValue();
    inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();
    inputs.driveTargetVelocity = Units.RadiansPerSecond.zero();

    inputs.turnPosition = Rotation2d.fromRotations(turnMotor.getEncoder().getPosition());
    inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.getValueAsDouble());
    inputs.turnVelocity = Units.RPM.of(turnMotor.getEncoder().getVelocity());
    inputs.turnAppliedVolts = Units.Volts.of(turnMotor.getAppliedOutput() * turnMotor.getBusVoltage());
    inputs.turnStatorCurrent = Units.Amps.of(turnMotor.getOutputCurrent());
    inputs.turnSupplyCurrent = Units.Amps.of(turnMotor.getOutputCurrent());
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveMotor.setControl(openLoopDriveRequest.withOutput(volts.in(Units.Volts)));
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    turnMotor.setVoltage(volts.in(Units.Volts));
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnPidController.setReference(
        position.getRotations(),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turnFF.getKS());
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocity) {
    setDriveVelocity(velocity, Units.Amps.zero());
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocity, Current torqueCurrent) {
    double velocityMetersPerSec = velocity.in(Units.MetersPerSecond);
    double velocityRadPerSec = velocityMetersPerSec / wheelRadiusMeters;
    driveMotor.setControl(closedLoopDriveRequest.withVelocity(velocityRadPerSec));
  }

  @Override
  public void reset() {
    driveMotor.setPosition(0.0);
  }

  @Override
  public void stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  @Override
  public void setDriveBrake(boolean enabled) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    driveMotor.getConfigurator().refresh(config);
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrake(boolean enabled) {
    turnMotor.configure(
        enabled ? turnBrakeConfig : turnCoastConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setDrivePID(PIDGains gains) {
    Slot0Configs config = new Slot0Configs();
    driveMotor.getConfigurator().refresh(config);
    driveMotor
        .getConfigurator()
        .apply(config.withKP(gains.getKP()).withKI(gains.getKI()).withKD(gains.getKD()));
  }

  @Override
  public void setSteerPID(PIDGains gains) {
    // SparkMax PID handled via configuration
  }

  @Override
  public void setDriveCurrent(Current current) {
    driveMotor.setControl(openLoopTorqueRequest.withOutput(current));
  }

  private static SparkMaxConfig createTurnConfig(
      PIDGains gains,
      double gearing,
      boolean inverted,
      SparkBaseConfig.IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(gains.getKP(), gains.getKI(), gains.getKD());
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(-0.5, 0.5);
    config.encoder.positionConversionFactor(1.0 / gearing);
    config.encoder.velocityConversionFactor(60.0 / gearing);
    config.inverted(inverted);
    config.idleMode(idleMode);
    config.smartCurrentLimit(30);
    return config;
  }
}
