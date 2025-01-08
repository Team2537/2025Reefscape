package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

fun Translation2d.flip(): Translation2d {
    return Translation2d(
        2 * FieldConstants.fieldCenter.translation.x - this.x,
        2 * FieldConstants.fieldCenter.translation.y - this.y
    )
}

fun Pose2d.flip(): Pose2d {
    return Pose2d(
        this.translation.flip(),
        this.rotation.rotateBy(Rotation2d.fromDegrees(180.0))
    )
}

fun Translation3d.flip(): Translation3d {
    return Translation3d(
        2 * FieldConstants.fieldCenter.translation.x - this.x,
        2 * FieldConstants.fieldCenter.translation.y - this.y,
        this.z
    )
}

fun Pose3d.flip(): Pose3d {
    return Pose3d(
        this.translation.flip(),
        this.rotation.rotateBy(
            Rotation3d(Rotation2d.fromDegrees(180.0))
        )
    )
}

