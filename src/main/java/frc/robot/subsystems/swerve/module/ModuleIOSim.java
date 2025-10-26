package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;

/** Simulation implementation of a swerve module. */
public final class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveMotor;
  private final DCMotorSim turnMotor;
  private final SimpleMotorFeedforward driveFeedforward;
  private final SimpleMotorFeedforward turnFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private final double wheelRadiusMeters;

  public ModuleIOSim(
      FeedforwardGains driveFF,
      PIDGains driveGains,
      DCMotor driveMotorGearbox,
      double driveGearing,
      FeedforwardGains turnFF,
      PIDGains turnGains,
      DCMotor turnMotorGearbox,
      double turnGearing,
      double wheelRadiusMeters) {
    this.driveMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorGearbox, 0.025, driveGearing),
            driveMotorGearbox);
    this.turnMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorGearbox, 0.004, turnGearing),
            turnMotorGearbox);

    this.driveFeedforward =
        new SimpleMotorFeedforward(driveFF.getKS(), driveFF.getKV(), driveFF.getKA());
    this.turnFeedforward =
        new SimpleMotorFeedforward(turnFF.getKS(), turnFF.getKV(), turnFF.getKA());

    this.driveFeedback = new PIDController(driveGains.getKP(), driveGains.getKI(), driveGains.getKD());
    this.turnFeedback = new PIDController(turnGains.getKP(), turnGains.getKI(), turnGains.getKD());
    this.wheelRadiusMeters = wheelRadiusMeters;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.update(Robot.getUpdateRateSec());
    turnMotor.update(Robot.getUpdateRateSec());

    inputs.driveMotorConnected = true;
    inputs.turnMotorConnected = true;
    inputs.absoluteEncoderConnected = true;

    inputs.drivePositionMeters = driveMotor.getAngularPositionRad() * wheelRadiusMeters;
    inputs.driveVelocityMetersPerSec = driveMotor.getAngularVelocityRadPerSec() * wheelRadiusMeters;
    inputs.drivePositionRad = driveMotor.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveMotor.getInputVoltage();
    inputs.driveStatorCurrentAmps = driveMotor.getCurrentDrawAmps();
    inputs.driveSupplyCurrentAmps = driveMotor.getCurrentDrawAmps();

    inputs.turnPosition = Rotation2d.fromRadians(turnMotor.getAngularPositionRad());
    inputs.absoluteTurnPosition = inputs.turnPosition;
    inputs.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnMotor.getInputVoltage();
    inputs.turnStatorCurrentAmps = turnMotor.getCurrentDrawAmps();
    inputs.turnSupplyCurrentAmps = turnMotor.getCurrentDrawAmps();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setInputVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setInputVoltage(volts);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    double desiredRad = position.getRadians();
    double feedback = turnFeedback.calculate(turnMotor.getAngularPositionRad(), desiredRad);
    double feedforward = turnFeedforward.calculate(turnMotor.getAngularVelocityRadPerSec());
    turnMotor.setInputVoltage(feedback + feedforward);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    double velocityRadPerSec = velocityMetersPerSec / wheelRadiusMeters;
    double ff = driveFeedforward.calculate(velocityRadPerSec);
    double fb = driveFeedback.calculate(driveMotor.getAngularVelocityRadPerSec(), velocityRadPerSec);
    driveMotor.setInputVoltage(ff + fb);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double torqueCurrentAmps) {
    setDriveVelocity(velocityMetersPerSec);
  }

  @Override
  public void reset() {
    driveMotor.setState(0.0, driveMotor.getAngularVelocityRadPerSec());
  }

  @Override
  public void stop() {
    driveMotor.setInputVoltage(0.0);
    turnMotor.setInputVoltage(0.0);
  }
}
