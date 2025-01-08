package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Centimeter
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotation
import lib.math.units.inches
import lib.math.units.into
import lib.math.units.measuredIn
import lib.math.units.meters
import org.littletonrobotics.junction.Logger

object FieldConstants {
    val fieldWidth = (26.0 * 12.0) + 5.0 measuredIn Inches
    val fieldLength = (57.0 * 12.0) + 6.0 + 7.0 / 8 measuredIn Inches
    
    val fieldCenter = Pose2d(fieldLength / 2.0, fieldWidth / 2.0, Rotation2d())
    
    val reefCenterBlue =
        Translation2d(
            Meters.of(4.489325),
            Meters.of(4.025877)
        )
    
    enum class Side {
        LEFT,
        RIGHT
    }
    
    enum class Level {
        L1, L2, L3, L4
    }
    
    enum class ReefFace {
        S1, S2, S3, S4, S5, S6
    }
    
    fun getNodePose(level: Level, side: ReefFace, rack: Side): Pose3d {
        var x = reefCenterBlue.x.meters - 29.421141.inches
        var y = reefCenterBlue.y.meters
        
        var z = when (level) {
            Level.L1 -> Centimeter.of(46.0)
            Level.L2 -> Centimeter.of(81.0)
            Level.L3 -> Centimeter.of(121.0)
            Level.L4 -> Centimeter.of(183.0)
        }
        
        when (rack) {
            Side.LEFT -> y -= Inches.of(12.937968 / 2)
            Side.RIGHT -> y += Inches.of(12.937968 / 2)
        }
        
        var translation2d = Translation2d(x, y)
        
        val sideDelta = Rotation2d.fromDegrees(60.0)
        
//        translation2d = translation2d.plus(translation2d.minus(reefCenterBlue).rotateBy(Rotation2d.fromDegrees(sideDelta * side.ordinal)))
        
        return Pose3d(
            Translation3d(translation2d.x, translation2d.y, z into Meters),
            Rotation3d(0.0, Units.degreesToRadians(35.0), sideDelta.radians * side.ordinal)
        )
    }
    
    val nodes: List<List<Pose3d>> = ReefFace.values().map { reefFace ->
        Side.values().flatMap { side ->
            Level.values().map { level ->
                getNodePose(level, reefFace, side)
            }
        }
    }
    
    init {
        nodes.forEachIndexed { i, list ->
            Logger.recordOutput("S${i + 1} Nodes", *list.toTypedArray())
        }
    }
}