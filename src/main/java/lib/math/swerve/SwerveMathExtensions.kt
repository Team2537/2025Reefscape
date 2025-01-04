package lib.math.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState

operator fun SwerveModuleState.minus(other: SwerveModuleState): SwerveModuleState {
    val thisAsTranslation = Translation2d(this.speedMetersPerSecond, this.angle)
    val otherAsTranslation = Translation2d(other.speedMetersPerSecond, other.angle)

    val result = thisAsTranslation - otherAsTranslation
    return SwerveModuleState(result.norm, result.angle)
}
