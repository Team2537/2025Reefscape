package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.swerve.Drivebase;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Characterizes wheel radius by spinning in place and comparing yaw to wheel rotation. */
public final class WheelRadiusCharacterization extends edu.wpi.first.wpilibj2.command.CommandBase {
  private final Drivebase drivebase;
  private final Direction direction;
  private final Measure<AngularVelocity> characterizationSpeed;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double[] startWheelPositionsRad = new double[0];
  private double lastGyroYawRads = 0.0;
  private double gyroYawAccumRads = 0.0;

  public WheelRadiusCharacterization(Drivebase drivebase, Direction direction) {
    this(drivebase, direction, Units.RadiansPerSecond.of(1.0));
  }

  public WheelRadiusCharacterization(
      Drivebase drivebase, Direction direction, Measure<AngularVelocity> characterizationSpeed) {
    this.drivebase = drivebase;
    this.direction = direction;
    this.characterizationSpeed = characterizationSpeed;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    List<Measure<Angle>> wheelAngles = drivebase.getWheelRadiusCharacterizationAngles();
    startWheelPositionsRad = new double[wheelAngles.size()];
    for (int i = 0; i < wheelAngles.size(); i++) {
      startWheelPositionsRad[i] = wheelAngles.get(i).in(Units.Radians);
    }

    gyroYawAccumRads = 0.0;
    lastGyroYawRads = drivebase.getGyroInputs().getYaw().getRadians();
    omegaLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    drivebase.runWheelRadiusCharacterization(characterizationSpeed.times(direction.sign));

    double currentYaw = drivebase.getGyroInputs().getYaw().getRadians();
    gyroYawAccumRads += MathUtil.angleModulus(currentYaw - lastGyroYawRads);
    lastGyroYawRads = currentYaw;

    List<Measure<Angle>> wheelPositions = drivebase.getWheelRadiusCharacterizationAngles();
    double sumRad = 0.0;
    for (int i = 0; i < wheelPositions.size(); i++) {
      double current = wheelPositions.get(i).in(Units.Radians);
      sumRad += Math.abs(current - startWheelPositionsRad[i]);
    }
    double averageWheelPositionRad = wheelPositions.isEmpty() ? 0.0 : sumRad / wheelPositions.size();

    Logger.recordOutput(
        drivebase.getName() + "/radiusCharacterization/averageWheelPositionRad",
        averageWheelPositionRad);

    if (averageWheelPositionRad > 1e-6) {
      double currentEffectiveWheelRadiusMeters =
          (gyroYawAccumRads * Drivebase.Companion.getDrivebaseRadius().in(Units.Meters)
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
