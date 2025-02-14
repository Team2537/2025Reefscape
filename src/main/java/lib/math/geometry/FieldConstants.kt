package lib.math.geometry

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import lib.math.units.centi
import lib.math.units.inches
import lib.math.units.into
import lib.math.units.measuredIn
import lib.math.units.meters
import org.littletonrobotics.junction.Logger

/**
 * Constants for the field
 *
 * Everything is blue by default, use the various `.flip()` methods to get red field constants
 */
object FieldConstants {
    
    /** Unit safe field width */
    val fieldWidth = (26.0 * 12.0) + 5.0 measuredIn Inches
    
    /** Unit safe field length */
    val fieldLength = (57.0 * 12.0) + 6.0 + 7.0 / 8 measuredIn Inches
    
    /** Center of the field */
    val fieldCenter = Pose2d(fieldLength / 2.0, fieldWidth / 2.0, Rotation2d())

    val tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    
    // Access the reef and processor constants to initialize them
    init {
        Reef
        Processor
        Barge
    }
    
    object Reef {
        
        /** Center of the blue reef */
        val reefCenterBlue =
            Translation2d(
                Meters.of(4.489325),
                Meters.of(4.025877)
            )
        
        val reefHexagonRadius = 37.69.inches
        
        
        /** Circle centered around the reef, expanding 20 inches past its edge */
        val reefZone: Ellipse2d = Ellipse2d(
            reefCenterBlue,
            reefHexagonRadius + 20.inches,
        )
        
        val reefHexagonBlue = Hexagon2d(reefCenterBlue, reefHexagonRadius, Rotation2d.fromDegrees(30.0))
        val reefHexagonRed = reefHexagonBlue.flip()
        
        /**
         * Represents the left and right racks on each side of the reef
         */
        enum class Side {
            LEFT,
            RIGHT
        }
        
        /**
         * Represents the different levels of the reef
         *
         * @param height The height of center of the PVC pipe making up the racks on this level
         * @param angle the angle that the PVC pipe points
         * @param radius the distance from the center of the reef to the center of the PVC pipe
         *
         * @see [calcNodePose]
         */
        enum class Level(val height: Distance, val angle: Double, val radius: Distance) {
            L1(46.0.centi.meters, 35.0, 30.679296.inches),
            L2(31.218618.inches, 35.0, 30.679296.inches),
            L3(47.088688.inches, 35.0, 30.679296.inches),
            L4(71.994600.inches, 90.0, 30.738196.inches)
        }
        
        /**
         * Represents the different faces of the reef
         *
         * S1 is the face closest to the driver station, continues CCW around the reef
         */
        enum class ReefFace {
            S1, S2, S3, S4, S5, S6
        }
        
        /**
         * Calculates the pose of a node on the reef, should only be ran at initialization, to cache the poses of all nodes
         *
         * @param level The level of the reef
         * @param side The side of the reef
         * @param rack The rack on the side of the reef
         *
         * @return The pose of the node
         */
        private fun calcNodePose(level: Level, side: ReefFace, rack: Side): Pose3d {
            var x = reefCenterBlue.x.meters - level.radius
            var y = reefCenterBlue.y.meters
            
            when (rack) {
                Side.LEFT -> y -= Inches.of(12.937968 / 2)
                Side.RIGHT -> y += Inches.of(12.937968 / 2)
            }
            
            var translation2d = Translation2d(x, y)
            
            val sideDelta = Rotation2d.fromDegrees(60.0).times(side.ordinal.toDouble())
            
            translation2d = Translation2d(
                (sideDelta.cos * (translation2d.x - reefCenterBlue.x)) - (sideDelta.sin * (translation2d.y - reefCenterBlue.y)) + reefCenterBlue.x,
                (sideDelta.sin * (translation2d.x - reefCenterBlue.x)) + (sideDelta.cos * (translation2d.y - reefCenterBlue.y)) + reefCenterBlue.y
            )
            
            return Pose3d(
                Translation3d(translation2d.x, translation2d.y, level.height into Meters),
                Rotation3d(0.0, Units.degreesToRadians(level.angle), sideDelta.radians)
            )
        }
        
        /**
         * Represents a node on the reef
         *
         * @param pose The pose of the node
         * @param side The side of the reef
         * @param level The level of the reef
         * @param reefFace The face of the reef
         */
        data class Node(val pose: Pose3d, val side: Side, val level: Level, val reefFace: ReefFace)
        
        /** List of all nodes on the reef */
        private val nodes: List<Node> = Level.values().flatMap { level ->
            ReefFace.values().flatMap { reefFace ->
                Side.values().map { side ->
                    Node(calcNodePose(level, reefFace, side), side, level, reefFace)
                }
            }
        }
        
        /**
         * Gets a node on the reef
         *
         * @param level The level of the reef
         * @param reefFace The face of the reef
         * @param side The side of the reef
         *
         * @return The node
         * @see Node
         * @throws NoSuchElementException if the node does not exist
         */
        fun getNode(level: Level, reefFace: ReefFace, side: Side): Node {
            return nodes.first { it.level == level && it.reefFace == reefFace && it.side == side }
        }
        
        init {
            Logger.recordOutput("field/reef/reefCenterBlue", reefCenterBlue)
            Logger.recordOutput("field/reef/reefZone", *reefZone.cardinals.toTypedArray())
            Logger.recordOutput("field/reef/reefHexagonBlue", *reefHexagonBlue.vertices.map { Pose2d(it, Rotation2d()) }.toTypedArray())
            Logger.recordOutput("field/reef/reefHexagonRed", *reefHexagonRed.vertices.map { Pose2d(it, Rotation2d()) }.toTypedArray())
        }
    }
    
    object Processor {
        val processorHoleCenter = Pose3d(
            Translation3d(
                (221.726104.inches) + (14.0.inches),
                Inches.zero(),
                17.0.inches
            ),
            Rotation3d()
        )
        
        val processorZone: Rectangle2d =
            Rectangle2d(
                processorHoleCenter.toPose2d().plus(Transform2d(Inches.zero(), 15.0.inches, Rotation2d())),
                50.inches,
                30.0.inches,
            )
        
        init {
            Logger.recordOutput("field/processor/processorHoleCenter", processorHoleCenter)
            Logger.recordOutput("field/processor/processorZone", *processorZone.corners.toTypedArray())
        }
        
    }
    
    object Barge {
        
        val allianceWallToCage = 346.891642.inches
        
        /** First cage from the left while at the blue alliance station */
        val cage1 = Pose2d(
            Translation2d(
                allianceWallToCage,
                285.822375.inches,
            ),
            Rotation2d()
        )
        
        /** Second cage from the left while at the blue alliance station */
        val cage2 = Pose2d(
            Translation2d(
                allianceWallToCage,
                242.884958.inches,
            ),
            Rotation2d()
        )
        
        /** Third cage from the left while at the blue alliance station */
        val cage3 = Pose2d(
            Translation2d(
                allianceWallToCage,
                199.947458.inches,
            ),
            Rotation2d()
        )
        
        val cageZone: Rectangle2d = Rectangle2d(
            cage2,
            ((3*12) + 10.0).inches,
            ((12 * 12) + 2.5).inches
        )
        
        init {
            Logger.recordOutput("field/barge/cages", cage1, cage2, cage3)
            Logger.recordOutput("field/barge/cageZone", *cageZone.corners.toTypedArray())
        }
    }
}