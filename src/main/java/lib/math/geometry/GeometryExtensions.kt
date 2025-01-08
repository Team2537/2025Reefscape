package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d

fun Pose2d.to3d(): Pose3d {
    return Pose3d(x, y, 0.0, Rotation3d(rotation))
}