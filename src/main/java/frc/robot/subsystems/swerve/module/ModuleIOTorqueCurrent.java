package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import edu.wpi.first.units.measure.Voltage;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;

/** Module IO that uses torque-current control on the drive motor. */
public final class ModuleIOTorqueCurrent implements ModuleIO {
  private final TalonFX driveMotor;
  private final SparkMax turnMotor;
  private final SparkClosedLoopController turnPidController;
  private final CANcoder absoluteEncoder;
  private final double wheelRadiusMeters;
  private final FeedforwardGains turnFF;
  private final PIDGains turnPID;
  private final double turnGearing;

  private final VoltageOut openLoopDriveRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC openLoopTorqueRequest = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC closedLoopDriveRequest = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveSupplyVolts;
  private final StatusSignal<Voltage> driveMotorVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;
  private final StatusSignal<Angle> absoluteTurnPosition;

  public ModuleIOTorqueCurrent(
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
    this.turnPID = turnPID;
    this.turnGearing = turnGearing;

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
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig);

    drivePosition = driveMotor.getPosition().clone();
    driveVelocity = driveMotor.getVelocity().clone();
    driveSupplyVolts = driveMotor.getSupplyVoltage().clone();
    driveMotorVolts = driveMotor.getMotorVoltage().clone();
    driveStatorCurrent = driveMotor.getStatorCurrent().clone();
    driveSupplyCurrent = driveMotor.getSupplyCurrent().clone();
    driveTorqueCurrent = driveMotor.getTorqueCurrent().clone();

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.closedLoop.pid(turnPID.getKP(), turnPID.getKI(), turnPID.getKD());
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(0.0, 1.0);
    turnConfig.encoder.positionConversionFactor(1.0 / turnGearing);
    turnConfig.encoder.velocityConversionFactor(60.0 / turnGearing);
    turnConfig.inverted(turnInverted);
    turnConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    turnConfig.smartCurrentLimit(30);

    turnMotor = new SparkMax(turnID, SparkLowLevel.MotorType.kBrushless);
    turnMotor.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
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
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                    drivePosition,
                    driveVelocity,
                    driveSupplyVolts,
                    driveMotorVolts,
                    driveStatorCurrent,
                    driveSupplyCurrent,
                    driveTorqueCurrent)
                .isOK();

    inputs.turnMotorConnected = true;
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(absoluteTurnPosition).isOK();

    inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = inputs.driveVelocityRadPerSec * wheelRadiusMeters;
    inputs.drivePositionRad = drivePosition.getValueAsDouble();
    inputs.drivePositionMeters = inputs.drivePositionRad * wheelRadiusMeters;
    inputs.driveAppliedVolts = driveMotorVolts.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    inputs.turnPosition = Rotation2d.fromRotations(turnMotor.getEncoder().getPosition());
    inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.RPM.of(turnMotor.getEncoder().getVelocity()).in(Units.RadiansPerSecond);
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnStatorCurrentAmps = turnMotor.getOutputCurrent();
    inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(openLoopDriveRequest.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnPidController.setReference(
        position.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, turnFF.getKS());
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    setDriveVelocity(velocityMetersPerSec, 0.0);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double torqueCurrentAmps) {
    double velocityRadPerSec = velocityMetersPerSec / wheelRadiusMeters;
    AngularVelocity velocityMeasure = Units.RadiansPerSecond.of(velocityRadPerSec);
    Current ff = Units.Amps.of(torqueCurrentAmps);
    driveMotor.setControl(closedLoopDriveRequest.withVelocity(velocityMeasure).withFeedForward(ff));
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
    SparkBaseConfig.IdleMode mode = enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast;
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(turnPID.getKP(), turnPID.getKI(), turnPID.getKD());
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(0.0, 1.0);
    config.encoder.positionConversionFactor(1.0 / turnGearing);
    config.encoder.velocityConversionFactor(60.0 / turnGearing);
    config.inverted(turnMotor.getInverted());
    config.idleMode(mode);
    config.smartCurrentLimit(30);
    turnMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
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
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(gains.getKP(), gains.getKI(), gains.getKD());
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(0.0, 1.0);
    config.encoder.positionConversionFactor(1.0 / turnGearing);
    config.encoder.velocityConversionFactor(60.0 / turnGearing);
    turnMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveCurrent(double currentAmps) {
    driveMotor.setControl(openLoopTorqueRequest.withOutput(Units.Amps.of(currentAmps)));
  }
}
