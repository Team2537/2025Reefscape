package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

/**
 * Characterizes wheel radius by spinning in place and comparing yaw to wheel
 * rotation.
 */
public final class WheelRadiusCharacterization extends Command {
  private final Drive drive;
  private final Direction direction;
  private final AngularVelocity characterizationSpeed;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double[] startWheelPositionsRad = new double[0];
  private double lastGyroYawRads = 0.0;
  private double gyroYawAccumRads = 0.0;

  public WheelRadiusCharacterization(Drive drive, Direction direction) {
    this(drive, direction, Units.RadiansPerSecond.of(1.0));
  }

  public WheelRadiusCharacterization(
      Drive drive, Direction direction, AngularVelocity characterizationSpeed) {
    this.drive = drive;
    this.direction = direction;
    this.characterizationSpeed = characterizationSpeed;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startWheelPositionsRad = drive.getWheelRadiusCharacterizationPositions();

    gyroYawAccumRads = 0.0;
    lastGyroYawRads = drive.getPose().getRotation().getRadians();
    omegaLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, characterizationSpeed.in(Units.RadiansPerSecond) * direction.sign));

    double currentYaw = drive.getPose().getRotation().getRadians();
    gyroYawAccumRads += MathUtil.angleModulus(currentYaw - lastGyroYawRads);
    lastGyroYawRads = currentYaw;

    double[] wheelPositions = drive.getWheelRadiusCharacterizationPositions();
    double sumRad = 0.0;
    for (int i = 0; i < wheelPositions.length; i++) {
      sumRad += Math.abs(wheelPositions[i] - startWheelPositionsRad[i]);
    }
    double averageWheelPositionRad = wheelPositions.length == 0 ? 0.0 : sumRad / wheelPositions.length;

    Logger.recordOutput(
        drive.getName() + "/radiusCharacterization/averageWheelPositionRad",
        averageWheelPositionRad);

    if (averageWheelPositionRad > 1e-6) {
      double currentEffectiveWheelRadiusMeters = (gyroYawAccumRads * Drive.DRIVE_BASE_RADIUS
          / averageWheelPositionRad)
          * direction.sign;
      Logger.recordOutput(
          drive.getName() + "/radiusCharacterization/wheelRadius",
          currentEffectiveWheelRadiusMeters);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
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
