package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

val Translation2d.flipped: Translation2d
    get() {
        return Translation2d(
            2 * FieldConstants.center.translation.x - this.x,
            2 * FieldConstants.center.translation.y - this.y
        )
    }

val Pose2d.flipped: Pose2d
    get() {
        return Pose2d(
            this.translation.flipped,
            this.rotation.rotateBy(Rotation2d.fromDegrees(180.0))
        )
    }

val Translation3d.flipped: Translation3d
    get() {
        return Translation3d(
            2 * FieldConstants.center.translation.x - this.x,
            2 * FieldConstants.center.translation.y - this.y,
            this.z
        )
    }

fun Pose3d.flipped(): Pose3d {
    return Pose3d(
        this.translation.flipped,
        this.rotation.rotateBy(
            Rotation3d(Rotation2d.fromDegrees(180.0))
        )
    )
}

