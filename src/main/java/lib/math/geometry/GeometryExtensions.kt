package lib.math.geometry

import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation3d

fun Pose2d.to3d(): Pose3d {
    return Pose3d(x, y, 0.0, Rotation3d(rotation))
}

fun Rectangle2d.corners(): List<Pose2d> {
    val x = center.x
    val y = center.y
    val w = xWidth / 2.0
    val h = yWidth / 2.0
    return listOf(
        Pose2d(x - w, y - h, rotation),
        Pose2d(x + w, y - h, rotation),
        Pose2d(x + w, y + h, rotation),
        Pose2d(x - w, y + h, rotation)
    )
}

fun Ellipse2d.cardinals(): List<Pose2d> {
    val x = center.x
    val y = center.y
    val a = xSemiAxis
    val b = ySemiAxis
    
    val directions = listOf(
        Math.PI / 2, // N
        Math.PI / 4, // NE
        0.0,         // E
        -Math.PI / 4, // SE
        -Math.PI / 2, // S
        -3 * Math.PI / 4, // SW
        Math.PI,     // W
        3 * Math.PI / 4  // NW
    )
    
    return directions.map { theta ->
        val dx = a * Math.cos(theta)
        val dy = b * Math.sin(theta)
        Pose2d(x + dx, y + dy, rotation)
    }
}
