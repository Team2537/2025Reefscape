package lib.math.geometry

import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.MatchType
import kotlin.jvm.optionals.getOrNull

typealias JavaDriverStation = edu.wpi.first.wpilibj.DriverStation

object DriverStation {
    /**
     * The current [Alliance] of the robot, or `null` if there is no current valid
     * alliance.
     *
     * @see edu.wpi.first.wpilibj.DriverStation.getAlliance()
     */
    val alliance: Alliance? get() = JavaDriverStation.getAlliance().getOrNull()

    // TODO: the rest of driver station stuff
}

/**
 * Returns a copy of this translation from the perspective of the other team.
 *
 * @return This translation as see by the other team.
 */
fun Translation2d.flipped(): Translation2d = Translation2d(
    2 * FieldConstants.fieldCenter.translation.x - this.x,
    2 * FieldConstants.fieldCenter.translation.y - this.y
)

/**
 * Returns a copy of this pose from the perspective of the other team.
 *
 * @return This pose as see by the other team.
 */
fun Pose2d.flipped(): Pose2d = Pose2d(
    this.translation.flipped(),
    this.rotation.rotateBy(Rotation2d.fromDegrees(180.0))
)

/**
 * Returns a copy of this translation from the perspective of the other team.
 *
 * @return This translation as see by the other team.
 */
fun Translation3d.flipped(): Translation3d = Translation3d(
    2 * FieldConstants.fieldCenter.translation.x - this.x,
    2 * FieldConstants.fieldCenter.translation.y - this.y,
    this.z
)

/**
 * Returns a copy of this pose from the perspective of the other team.
 *
 * @return This pose as see by the other team.
 */
fun Pose3d.flipped(): Pose3d = Pose3d(
    this.translation.flipped(),
    this.rotation.rotateBy(
        Rotation3d(Rotation2d.fromDegrees(180.0))
    )
)

/**
 * Returns a copy of this rectangle from the perspective of the other team.
 *
 * @return This rectangle as seen by the other team.
 */
fun Rectangle2d.flipped(): Rectangle2d = Rectangle2d(center.flipped(), xWidth, yWidth)

/**
 * Returns a copy of this ellipse from the perspective of the other team.
 *
 * @return This ellipse as seen by the other team.
 */
fun Ellipse2d.flipped(): Ellipse2d = Ellipse2d(center.flipped(), xSemiAxis, ySemiAxis)
