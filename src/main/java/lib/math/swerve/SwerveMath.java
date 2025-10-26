package lib.math.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Utility math for swerve modules. */
public final class SwerveMath {
  private SwerveMath() {}

  public static SwerveModuleState subtract(SwerveModuleState a, SwerveModuleState b) {
    Translation2d aTranslation = new Translation2d(a.speedMetersPerSecond, a.angle);
    Translation2d bTranslation = new Translation2d(b.speedMetersPerSecond, b.angle);
    Translation2d result = aTranslation.minus(bTranslation);
    return new SwerveModuleState(result.getNorm(), result.getAngle());
  }
}
