package lib.controllers.pathfollowing.repulsor

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.units.into
import org.ejml.equation.IntegerSequence.For
import java.util.*
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow

class FieldPlanner {
    
    var goal: Optional<Translation2d> = Optional.empty()
        private set
    
    val arrows: MutableList<Pose2d> = List(200, { Pose2d() }).toMutableList()
    
    fun getGoalForce(currLocation: Translation2d): Force {
        return goal.map { goalLoc ->
            val displacement = goalLoc - currLocation
            if (displacement.norm == 0.0) {
                return@map Force()
            }
            val direction = displacement.angle
            
            val mag = GOAL_STRENGTH * (1 + 1.0 / (0.00001 + displacement.norm))
            return@map Force(mag, direction)
        }.orElse(Force())
    }
    
    fun getForce(currLocation: Translation2d): Force {
        var goalForce = getGoalForce(currLocation)
        obstacles.forEachIndexed { i, obstacle ->
            val force = obstacle.getForceAtPosition(currLocation)
            goalForce += force
        }
        return goalForce
    }
    
    fun setGoal(goal: Translation2d) {
        this.goal = Optional.of(goal)
        updateArrows()
    }
    
    fun clearGoal() {
        goal = Optional.empty()
    }
    
    fun updateArrows() {
        for (x in 0..19) {
            for (y in 0..9) {
                val translation = Translation2d(
                    x * FieldConstants.fieldLength.into(Meters) / 20.0,
                    y * FieldConstants.fieldWidth.into(Meters) / 10.0
                )
                
                val rotation = getForce(translation).angle
                
                arrows[x * 10 + y] = Pose2d(translation, rotation)
            }
        }
    }
    
    companion object {
        const val GOAL_STRENGTH = 1.0
        
        abstract class Obstacle(
            val strength: Double = 1.0,
            val positive: Boolean = true
        ) {
            abstract fun getForceAtPosition(position: Translation2d): Force
            protected fun distToForceMag(dist: Double): Double {
                val forceMag = strength / (0.00001 + dist.pow(2).absoluteValue)
                return if (positive) forceMag else -forceMag
            }
        }
        
        class PointObstacle(
            val loc: Translation2d,
            strength: Double = 1.0,
            positive: Boolean = true
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(position: Translation2d): Force {
                return Force(distToForceMag(loc.getDistance(position)), (position - loc).angle)
            }
        }
        
        class HorizontalWallObstacle(
            val y: Double,
            strength: Double = 1.0,
            positive: Boolean = true
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(position: Translation2d): Force {
                return Force(0.0, distToForceMag(y - position.y))
            }
        }
        
        class VerticalWallObstacle(
            val x: Double,
            strength: Double = 1.0,
            positive: Boolean = true
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(position: Translation2d): Force {
                return Force(distToForceMag(x - position.x), 0.0)
            }
        }
        
        val obstacles = listOf(
            HorizontalWallObstacle(0.0, 0.5, true),
            HorizontalWallObstacle(FieldConstants.fieldWidth.into(Meters), 0.5, false),
            VerticalWallObstacle(0.0, 0.5, true),
            VerticalWallObstacle(FieldConstants.fieldLength.into(Meters), 0.5, false),
            PointObstacle(FieldConstants.Reef.reefCenterBlue, 1.0, true),
            PointObstacle(FieldConstants.Reef.reefCenterBlue.flipped(), 1.0, true),
        )
    }
}