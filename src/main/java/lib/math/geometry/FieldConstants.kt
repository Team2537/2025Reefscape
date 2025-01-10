package lib.math.geometry

import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import lib.math.geometry.FieldConstants.Processor.blueHoleCenter
import lib.math.geometry.FieldConstants.Processor.currentHoleCenter
import lib.math.geometry.FieldConstants.Processor.redHoleCenter
import lib.math.units.*
import org.littletonrobotics.junction.Logger

/**
 * Constants for the field
 *
 * Everything is blue by default, use the various `.flip()` methods to get red field constants
 */
object FieldConstants {
    
    /** Unit safe field width */
    val width = (26.0 * 12.0) + 5.0 measuredIn Inches
    
    /** Unit safe field length */
    val length = (57.0 * 12.0) + 6.0 + 7.0 / 8 measuredIn Inches
    
    /** Center of the field */
    val center = Pose2d(length / 2.0, width / 2.0, Rotation2d())
    
    // Access the reef and processor constants to initialize them
    init {
        Reef
        Processor
        Barge
    }
    
    object Reef {
        /** The center of the reef */
        val center: Translation2d get() = currentCenter

        /** Center of the blue reef */
        val blueCenter =
            Translation2d(
                Meters.of(4.489325),
                Meters.of(4.025877)
            )

        /** Center of the red reef */
        val redCenter = blueCenter.flipped

        private var currentCenter = blueCenter;

        /**
         * Flips the perspective of the [Reef] object from red to blue
         * or vice versa.
         */
        @Synchronized
        fun flip() {
            if (_isBlue) {
                currentCenter = redCenter
                currentZone = redZone
                currentNodes = redNodes
            } else {
                currentCenter = blueCenter
                currentZone = blueZone
                currentNodes = blueNodes
            }
            _isBlue = !_isBlue
        }

        private var _isBlue: Boolean = true
        val isBlue: Boolean get() = _isBlue
        val isRed: Boolean get() = !_isBlue

        /** Circle centered around the reef, expanding 20 inches past its edge */
        val zone: Ellipse2d get() = currentZone;

        /** The circular zone for the blue reef */
        val blueZone: Ellipse2d = Ellipse2d(
            blueCenter,
            32.745545.inches + 20.inches,
        );

        /** The circular zone for the red reef */
        val redZone: Ellipse2d = blueZone.flipped;

        private var currentZone: Ellipse2d = blueZone

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
        enum class Level(val height: Distance, val angle: Angle, val radius: Distance) {
            L1(46.0.centi.meters, 35.0.degrees, 30.679296.inches),
            L2(31.218618.inches, 35.0.degrees, 30.679296.inches),
            L3(47.088688.inches, 35.0.degrees, 30.679296.inches),
            L4(71.994600.inches, 90.0.degrees, 30.738196.inches)
        }
        
        /**
         * Represents the different faces of the reef
         *
         * S1 is the face closest to the driver station, continues CCW around the reef
         */
        enum class Face {
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
        private fun calcNodePose(level: Level, side: Face, rack: Side): Pose3d {
            var x = center.x.meters - level.radius
            var y = center.y.meters
            
            when (rack) {
                Side.LEFT -> y -= Inches.of(12.937968 / 2)
                Side.RIGHT -> y += Inches.of(12.937968 / 2)
            }
            
            var translation2d = Translation2d(x, y)
            
            val sideDelta = Rotation2d.fromDegrees(60.0).times(side.ordinal.toDouble())
            
            translation2d = Translation2d(
                (sideDelta.cos * (translation2d.x - center.x)) - (sideDelta.sin * (translation2d.y - center.y)) + center.x,
                (sideDelta.sin * (translation2d.x - center.x)) + (sideDelta.cos * (translation2d.y - center.y)) + center.y
            )
            
            return Pose3d(
                Translation3d(translation2d.x, translation2d.y, level.height into Meters),
                Rotation3d(0.0, level.angle into Radians, sideDelta.radians)
            )
        }
        
        /**
         * Represents a node on the reef
         *
         * @param pose The pose of the node
         * @param side The side of the reef
         * @param level The level of the reef
         * @param face The face of the reef
         */
        data class Node(val pose: Pose3d, val side: Side, val level: Level, val face: Face)
        
        /** List of all nodes on the reef */
        private val nodes: List<Node> get() = currentNodes
        private var currentNodes: List<Node>
        private val blueNodes: List<Node>
        private val redNodes: List<Node>

        /**
         * Gets a node on the reef
         *
         * @param level The level of the reef
         * @param face The face of the reef
         * @param side The side of the reef
         *
         * @return The node
         * @see Node
         * @throws NoSuchElementException if the node does not exist
         */
        fun getNode(level: Level, face: Face, side: Side): Node {
            return nodes.first { it.level == level && it.face == face && it.side == side }
        }
        
        init {
            // get blue nodes
            blueNodes = makeNodeList()

            // flip to make red nodes then flip back
            flip()
            redNodes = makeNodeList()
            flip()

            currentNodes = blueNodes

            Logger.recordOutput("field/reef/blueCenter", blueCenter)
            Logger.recordOutput("field/reef/blueZone", *blueZone.cardinals.toTypedArray())
        }

        private fun makeNodeList() = Level.entries.flatMap { level ->
            Face.entries.flatMap { reefFace ->
                Side.entries.map { side ->
                    Node(calcNodePose(level, reefFace, side), side, level, reefFace)
                }
            }
        }
    }
    
    object Processor {
        private var _isBlue: Boolean = true
        val isBlue: Boolean get() = _isBlue
        val isRed: Boolean get() = !_isBlue

        fun flip() {
            if(_isBlue) {
                currentHoleCenter = redHoleCenter
                currentZone = redZone
            } else {
                currentHoleCenter = blueHoleCenter
                currentZone = blueZone
            }
            _isBlue = !_isBlue
        }

        val holeCenter: Pose3d get() = currentHoleCenter

        private val blueHoleCenter: Pose3d = Pose3d(
            Translation3d(
                (221.726104.inches) + (14.0.inches),
                Inches.zero(),
                17.0.inches
            ),
            Rotation3d()
        )
        private val redHoleCenter: Pose3d = blueHoleCenter.flipped
        private var currentHoleCenter = blueHoleCenter

        val zone: Rectangle2d get() = currentZone

        private val blueZone: Rectangle2d = Rectangle2d(
            holeCenter.toPose2d() + Transform2d(Inches.zero(), 15.0.inches, Rotation2d()),
            50.inches,
            30.inches,
        )
        private val redZone: Rectangle2d = blueZone.flipped
        private var currentZone: Rectangle2d = blueZone

        init {
            Logger.recordOutput("field/processor/holeCenter", holeCenter)
            // Log corners to display them; default rect2d logging is less intuitive for this
            Logger.recordOutput("field/processor/zone", *zone.corners.toTypedArray())
        }
        
    }
    
    object Barge {
        private var _isBlue: Boolean = true
        val isBlue: Boolean get() = _isBlue
        val isRed: Boolean get() = !_isBlue

        fun flip() {
            if(_isBlue) {
                currentCage1 = redCage1
                currentCage2 = redCage2
                currentCage3 = redCage3
                currentZone = redZone
            } else {
                currentCage1 = blueCage1
                currentCage2 = blueCage2
                currentCage3 = blueCage3
                currentZone = blueZone

            }
            _isBlue = !_isBlue
        }

        
        val allianceWallToCage = 346.891642.inches
        
        /** First cage from the left while at the blue alliance station */
        val cage1: Pose2d get() = currentCage1
        private val blueCage1: Pose2d = Pose2d(
            Translation2d(
                allianceWallToCage,
                285.822375.inches,
            ),
            Rotation2d()
        )
        private val redCage1: Pose2d = blueCage1.flipped
        private var currentCage1: Pose2d = blueCage1

        /** Second cage from the left while at the blue alliance station */
        val cage2: Pose2d get() = currentCage2
        private val blueCage2: Pose2d = Pose2d(
            Translation2d(
                allianceWallToCage,
                242.884958.inches,
            ),
            Rotation2d()
        )
        private val redCage2: Pose2d = blueCage2.flipped
        private var currentCage2: Pose2d = blueCage2

        /** Third cage from the left while at the blue alliance station */
        val cage3: Pose2d get() = currentCage3
        private val blueCage3: Pose2d = Pose2d(
            Translation2d(
                allianceWallToCage,
                199.947458.inches,
            ),
            Rotation2d()
        )
        private val redCage3: Pose2d = blueCage3.flipped
        private var currentCage3: Pose2d = blueCage3

        val zone: Rectangle2d get() = currentZone
        private val blueZone: Rectangle2d = Rectangle2d(
            cage2,
            ((3*12) + 10.0).inches,
            ((12 * 12) + 2.5).inches
        )
        private val redZone: Rectangle2d = blueZone.flipped
        private var currentZone: Rectangle2d = blueZone

        init {
            Logger.recordOutput("field/barge/cages", cage1, cage2, cage3)
            Logger.recordOutput("field/barge/zone", *zone.corners.toTypedArray())
        }
    }
}