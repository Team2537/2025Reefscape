package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;

/** Real robot module IO layer that uses Talon FX drive with Talon FXS steering. */
public final class ModuleIOHybridFXS implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFXS turnMotor;
  private final CANcoder absoluteEncoder;
  private final double wheelRadiusMeters;

  private final VoltageOut openLoopDriveRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC openLoopTorqueRequest = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC closedLoopDriveRequest = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage closedLoopTurnRequest = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveSupplyVolts;
  private final StatusSignal<Voltage> driveMotorVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVoltage;
  private final StatusSignal<Current> turnSupplyCurrent;
  private final StatusSignal<Current> turnStatorCurrent;

  private final StatusSignal<Angle> absoluteTurnPosition;

  public ModuleIOHybridFXS(
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

    driveMotor = new TalonFX(driveID, "canivore");
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
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig);

    drivePosition = driveMotor.getPosition().clone();
    driveVelocity = driveMotor.getVelocity().clone();
    driveSupplyVolts = driveMotor.getSupplyVoltage().clone();
    driveMotorVolts = driveMotor.getMotorVoltage().clone();
    driveStatorCurrent = driveMotor.getStatorCurrent().clone();
    driveSupplyCurrent = driveMotor.getSupplyCurrent().clone();
    driveTorqueCurrent = driveMotor.getTorqueCurrent().clone();

    turnMotor = new TalonFXS(turnID, "canivore");
    TalonFXSConfiguration turnConfig = new TalonFXSConfiguration();
    turnConfig.ExternalFeedback.RotorToSensorRatio = turnGearing;
    turnConfig.ExternalFeedback.FeedbackRemoteSensorID = encoderID;
    turnConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    turnConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted =
        turnInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turnConfig.Slot0.kP = turnPID.getKP();
    turnConfig.Slot0.kI = turnPID.getKI();
    turnConfig.Slot0.kD = turnPID.getKD();
    turnConfig.Slot0.kV = turnFF.getKV();
    turnConfig.Slot0.kA = turnFF.getKA();
    turnConfig.Slot0.kS = turnFF.getKS();
    turnConfig.MotionMagic.withMotionMagicCruiseVelocity(Units.RPM.of(260.0));
    turnConfig.MotionMagic.withMotionMagicAcceleration(Units.RotationsPerSecondPerSecond.of(500.0));
    turnMotor.getConfigurator().apply(turnConfig);

    turnPosition = turnMotor.getPosition().clone();
    turnVelocity = turnMotor.getVelocity().clone();
    turnAppliedVoltage = turnMotor.getMotorVoltage().clone();
    turnSupplyCurrent = turnMotor.getSupplyCurrent().clone();
    turnStatorCurrent = turnMotor.getStatorCurrent().clone();

    absoluteEncoder = new CANcoder(encoderID, "canivore");
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    absoluteEncoder.getConfigurator().apply(encoderConfig);
    absoluteTurnPosition = absoluteEncoder.getAbsolutePosition().clone();
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

    inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
                    turnPosition,
                    turnVelocity,
                    turnSupplyCurrent,
                    turnAppliedVoltage,
                    turnStatorCurrent)
                .isOK();

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

    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.absoluteTurnPosition =
        Rotation2d.fromRotations(absoluteTurnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVoltage.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
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
    turnMotor.setControl(closedLoopTurnRequest.withPosition(position.getRotations()));
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
    MotorOutputConfigs config = new MotorOutputConfigs();
    turnMotor.getConfigurator().refresh(config);
    turnMotor
        .getConfigurator()
        .apply(config.withNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast));
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
    Slot0Configs config = new Slot0Configs();
    turnMotor.getConfigurator().refresh(config);
    turnMotor
        .getConfigurator()
        .apply(config.withKP(gains.getKP()).withKI(gains.getKI()).withKD(gains.getKD()));
  }

  @Override
  public void setDriveCurrent(double currentAmps) {
    driveMotor.setControl(openLoopTorqueRequest.withOutput(Units.Amps.of(currentAmps)));
  }
}
