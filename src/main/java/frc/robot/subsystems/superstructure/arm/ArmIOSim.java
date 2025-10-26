package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.superstructure.SuperstructureGoals;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;

/** Simulation arm IO using WPILib SingleJointedArmSim. */
public final class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim;
  private final PIDController pid;
  private final ArmFeedforward feedforward;

  private boolean positionControl = false;
  private Rotation2d targetAngle = Rotation2d.fromDegrees(0.0);
  private double appliedVolts = 0.0;

  public ArmIOSim() {
    this(
        ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO,
        0.15,
        0.35,
        new PIDGains(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD),
        new FeedforwardGains(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA));
  }

  public ArmIOSim(
      double gearing,
      double momentOfInertiaKgM2,
      double armLengthMeters,
      PIDGains pidGains,
      FeedforwardGains ffGains) {
    this.sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            gearing,
            momentOfInertiaKgM2,
            armLengthMeters,
            ArmConstants.MIN_ANGLE.getRadians(),
            ArmConstants.MAX_ANGLE.getRadians(),
            true,
            SuperstructureGoals.STOW.getArmAngle().getRadians());
    this.pid = new PIDController(pidGains.kP(), pidGains.kI(), pidGains.kD());
    this.feedforward = new ArmFeedforward(ffGains.kS(), ffGains.kG(), ffGains.kV(), ffGains.kA());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (positionControl) {
      double pidOutput = pid.calculate(sim.getAngleRads(), targetAngle.getRadians());
      double ffOutput = feedforward.calculate(targetAngle.getRadians(), 0.0);
      appliedVolts = pidOutput + ffOutput;
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
    inputs.angle = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.angularVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.leftStatorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.rightStatorCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    positionControl = false;
    appliedVolts = volts;
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    positionControl = true;
    targetAngle = angle;
    pid.reset();
  }

  @Override
  public void setBrakeMode(boolean brake) {
    // No-op for simulation
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  public record PIDGains(double kP, double kI, double kD) {}

  public record FeedforwardGains(double kS, double kG, double kV, double kA) {}
}
