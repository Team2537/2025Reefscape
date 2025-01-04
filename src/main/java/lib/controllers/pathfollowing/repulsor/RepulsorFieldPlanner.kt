package lib.controllers.pathfollowing.repulsor

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.RobotBase
import frc.robot.Robot
import java.util.Optional
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.sign

class RepulsorFieldPlanner {
    private val obstacles = FIELD_OBSTACLES + WALLS
    private var goalOpt: Optional<Translation2d> = Optional.empty()

    var goal: Translation2d
        get() = goalOpt.orElse(Translation2d.kZero)
        set(value) {
            goalOpt = Optional.of(value)
            updateArrows()
        }

    val arrows: MutableList<Pose2d> = MutableList(VERTICAL_ARROW_COUNT * HORIZONTAL_ARROW_COUNT) { Pose2d() }

    fun updateArrows() {
        if (RobotBase.isSimulation()) {
            for (col in 0..<HORIZONTAL_ARROW_COUNT) {
                for (row in 0..<VERTICAL_ARROW_COUNT) {
                    val translation = Translation2d(col * FIELD_LENGTH / HORIZONTAL_ARROW_COUNT, row * FIELD_WIDTH / VERTICAL_ARROW_COUNT)
                    val rotation = getForce(translation).angle
                    arrows[col * VERTICAL_ARROW_COUNT + row] = Pose2d(translation, rotation)
                }
            }
        }
    }

    fun getForceToGoal(position: Translation2d): Force {
        return goalOpt.map { goal ->
            val displacement = goal - position
            if (displacement.norm < 0.0001) {
                return@map Force()
            }

            val angle = displacement.angle
            val mag = GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.norm))

            Force(mag, angle)
        }.orElse(Force())
    }

    fun getForce(position: Translation2d): Force {
        var goalForce = getForceToGoal(position)
        obstacles.forEach { obs ->
            goalForce += obs.getForceAtPosition(position, goal)
        }
        return goalForce
    }

    fun getSample(
        position: Translation2d,
        rot: Rotation2d,
        vx: Double,
        vy: Double,
        omega: Double,
    ): SwerveSample {
        return SwerveSample(
            0.0,
            position.x,
            position.y,
            rot.radians,
            vx, vy, omega,
            0.0, 0.0, 0.0,
            doubleArrayOf(),
            doubleArrayOf(),
        )
    }

    fun getNextSample(
        currPose: Supplier<Pose2d>,
        stepSizeMeters: Double,
    ): SwerveSample {
        val curr = currPose.get()
        if (goalOpt.isEmpty) {
            return getSample(curr.translation, curr.rotation, 0.0, 0.0, 0.0)
        } else {
            val goal = goalOpt.get()
            val currTrans = curr.translation
            val err = currTrans - goal
            if (err.norm < stepSizeMeters * 1.5) {
                return getSample(goal, curr.rotation, 0.0, 0.0, 0.0)
            } else {
                val netForce = getForce(currTrans)
                val step = Translation2d(stepSizeMeters, netForce.angle)
                val intermediateGoal = currTrans + step
                return getSample(intermediateGoal, curr.rotation, step.x / 0.02, step.y / 0.02, 0.0)
            }
        }
    }

    companion object {
        const val GOAL_STRENGTH = 1.0
        const val STAGE_POINT_STRENGTH = 0.7

        val FIELD_OBSTACLES =
            listOf(
                DodgedPointObstacle(Translation2d(5.56, 2.74), STAGE_POINT_STRENGTH, true),
                DodgedPointObstacle(Translation2d(3.45, 4.07), STAGE_POINT_STRENGTH, true),
                DodgedPointObstacle(Translation2d(5.56, 5.35), STAGE_POINT_STRENGTH, true),
                DodgedPointObstacle(Translation2d(11.0, 2.74), STAGE_POINT_STRENGTH, true),
                DodgedPointObstacle(Translation2d(13.27, 4.07), STAGE_POINT_STRENGTH, true),
                DodgedPointObstacle(Translation2d(11.0, 5.35), STAGE_POINT_STRENGTH, true),
            )

        val FIELD_LENGTH = Robot.field.fieldLength
        val FIELD_WIDTH = Robot.field.fieldWidth

        const val HORIZONTAL_ARROW_COUNT = 20
        const val VERTICAL_ARROW_COUNT = 10

        val WALLS =
            listOf(
                HorizontalObstacle(0.0, 0.5, true),
                HorizontalObstacle(FIELD_WIDTH, 0.5, false),
                VerticalObstacle(0.0, 0.5, true),
                VerticalObstacle(FIELD_LENGTH, 0.5, false),
            )

        abstract class Obstacle(val strength: Double = 1.0, val positive: Boolean = true) {
            abstract fun getForceAtPosition(
                position: Translation2d,
                target: Translation2d,
            ): Force

            protected fun forceMagnitudeFromDistance(distance: Double): Double {
                val forceMag = strength / (0.00001 + abs(distance * distance))
                return if (positive) forceMag else -forceMag
            }
        }

        class PointObstacle(
            val loc: Translation2d,
            strength: Double = 1.0,
            positive: Boolean = true,
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(
                position: Translation2d,
                target: Translation2d,
            ): Force {
                return Force(
                    forceMagnitudeFromDistance(loc.getDistance(position)),
                    position.minus(loc).angle,
                )
            }
        }

        class DodgedPointObstacle(
            val loc: Translation2d,
            strength: Double = 1.0,
            positive: Boolean = true,
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(
                position: Translation2d,
                target: Translation2d,
            ): Force {
                val dist = loc.getDistance(position)
                if (dist > 4) return Force()
                val targetToLoc = loc - target
                val targetToLocAngle = targetToLoc.angle

                // Place the dodge point 1 meter away from the obstacle opposite the target
                val dodgePoint = loc + Translation2d(1.0, targetToLocAngle)
                val sidewaysMag = forceMagnitudeFromDistance(dodgePoint.getDistance(position)) / 2
                val outwardsMag = forceMagnitudeFromDistance(loc.getDistance(position))

                val initial =
                    Force(
                        outwardsMag,
                        (position - loc).angle,
                    )

                val sidewaysTheta = (target - position).angle - (position - dodgePoint).angle
                val sideways = sidewaysMag * sign(sidewaysTheta.sin)

                val sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg)

                return initial +
                    Force(
                        sideways,
                        sidewaysAngle,
                    )
            }
        }

        class HorizontalObstacle(
            val y: Double,
            strength: Double = 1.0,
            positive: Boolean = true,
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(
                position: Translation2d,
                target: Translation2d,
            ): Force {
                return Force(
                    0.0,
                    forceMagnitudeFromDistance(y - position.y),
                )
            }
        }

        class VerticalObstacle(
            val x: Double,
            strength: Double = 1.0,
            positive: Boolean = true,
        ) : Obstacle(strength, positive) {
            override fun getForceAtPosition(
                position: Translation2d,
                target: Translation2d,
            ): Force {
                return Force(
                    forceMagnitudeFromDistance(x - position.x),
                    0.0,
                )
            }
        }
    }
}
