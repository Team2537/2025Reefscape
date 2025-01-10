package lib.math.geometry

import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

/**
 * Returns a copy of this translation from the perspective of the other team.
 *
 * @return This translation as see by the other team.
 */
val Translation2d.flipped: Translation2d
    get() = Translation2d(
        2 * FieldConstants.center.translation.x - this.x,
        2 * FieldConstants.center.translation.y - this.y
    )

/**
 * Returns a copy of this pose from the perspective of the other team.
 *
 * @return This pose as see by the other team.
 */
val Pose2d.flipped: Pose2d
    get() = Pose2d(
        this.translation.flipped,
        this.rotation.rotateBy(Rotation2d.fromDegrees(180.0))
    )

/**
 * Returns a copy of this translation from the perspective of the other team.
 *
 * @return This translation as see by the other team.
 */
val Translation3d.flipped: Translation3d
    get() = Translation3d(
        2 * FieldConstants.center.translation.x - this.x,
        2 * FieldConstants.center.translation.y - this.y,
        this.z
    )

/**
 * Returns a copy of this pose from the perspective of the other team.
 *
 * @return This pose as see by the other team.
 */
val Pose3d.flipped: Pose3d
    get() = Pose3d(
        this.translation.flipped,
        this.rotation.rotateBy(
            Rotation3d(Rotation2d.fromDegrees(180.0))
        )
    )

/**
 * Returns a copy of this rectangle from the perspective of the other team.
 *
 * @return This rectangle as seen by the other team.
 */
val Rectangle2d.flipped: Rectangle2d
    get() = Rectangle2d(center.flipped, xWidth, yWidth)

/**
 * Returns a copy of this ellipse from the perspective of the other team.
 *
 * @return This ellipse as seen by the other team.
 */
val Ellipse2d.flipped: Ellipse2d
    get() = Ellipse2d(center.flipped, xSemiAxis, ySemiAxis)
