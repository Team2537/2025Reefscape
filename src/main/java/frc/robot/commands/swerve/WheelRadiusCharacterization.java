package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivebase;
import org.littletonrobotics.junction.Logger;

/** Characterizes wheel radius by spinning in place and comparing yaw to wheel rotation. */
public final class WheelRadiusCharacterization extends Command {
  private final Drivebase drivebase;
  private final Direction direction;
  private final AngularVelocity characterizationSpeed;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double[] startWheelPositionsRad = new double[0];
  private double lastGyroYawRads = 0.0;
  private double gyroYawAccumRads = 0.0;

  public WheelRadiusCharacterization(Drivebase drivebase, Direction direction) {
    this(drivebase, direction, Units.RadiansPerSecond.of(1.0));
  }

  public WheelRadiusCharacterization(
      Drivebase drivebase, Direction direction, AngularVelocity characterizationSpeed) {
    this.drivebase = drivebase;
    this.direction = direction;
    this.characterizationSpeed = characterizationSpeed;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    startWheelPositionsRad = drivebase.getWheelRadiusCharacterizationAngles();

    gyroYawAccumRads = 0.0;
    lastGyroYawRads = drivebase.getGyroInputs().yaw.getRadians();
    omegaLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    drivebase.runWheelRadiusCharacterization(characterizationSpeed.in(Units.RadiansPerSecond) * direction.sign);

    double currentYaw = drivebase.getGyroInputs().yaw.getRadians();
    gyroYawAccumRads += MathUtil.angleModulus(currentYaw - lastGyroYawRads);
    lastGyroYawRads = currentYaw;

    double[] wheelPositions = drivebase.getWheelRadiusCharacterizationAngles();
    double sumRad = 0.0;
    for (int i = 0; i < wheelPositions.length; i++) {
      sumRad += Math.abs(wheelPositions[i] - startWheelPositionsRad[i]);
    }
    double averageWheelPositionRad = wheelPositions.length == 0 ? 0.0 : sumRad / wheelPositions.length;

    Logger.recordOutput(
        drivebase.getName() + "/radiusCharacterization/averageWheelPositionRad",
        averageWheelPositionRad);

    if (averageWheelPositionRad > 1e-6) {
      double currentEffectiveWheelRadiusMeters =
          (gyroYawAccumRads * Drivebase.getDrivebaseRadiusMeters()
                  / averageWheelPositionRad)
              * direction.sign;
      Logger.recordOutput(
          drivebase.getName() + "/radiusCharacterization/wheelRadius",
          currentEffectiveWheelRadiusMeters);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.applyChassisSpeeds(new ChassisSpeeds());
  }

  public enum Direction {
    CLOCKWISE(-1.0),
    COUNTERCLOCKWISE(1.0);

    final double sign;

    Direction(double sign) {
      this.sign = sign;
    }
  }
}
