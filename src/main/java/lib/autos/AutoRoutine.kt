package lib.autos

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.swerve.Drivebase
import lib.math.geometry.FieldConstants.Reef
import lib.math.geometry.flipped
import kotlin.jvm.optionals.getOrDefault

class AutoRoutine(
    private val actions: List<Triple<Reef.Branch, Reef.Level, Boolean>>,
    private val drivebase: Drivebase,
    private val superstructure: Superstructure
) {

    fun build(): Command {
        val sequence = SequentialCommandGroup()



        sequence.addCommands(
            Commands.defer(
                {
                    runOnce({
                        var pose = getPathToBranch(actions.first().first, actions.first().third).startingHolonomicPose.getOrDefault(
                            Pose2d()
                        )

                        if (AutoBuilder.shouldFlip()) {
                            pose = pose.flipped()
                        }

                        drivebase.resetOdometry(pose)
                    })
                },
                setOf()
            ),
        )

        actions.forEachIndexed { index, (branch, level, isTop) ->
            sequence.addCommands(
                Commands.parallel(
                    AutoBuilder.followPath(getPathToBranch(branch, isTop)),
                    when (level) {
                        Reef.Level.L1 -> superstructure.getPrepL1Command()
                        Reef.Level.L2 -> superstructure.getPrepL2Command()
                        Reef.Level.L3 -> superstructure.getPrepL3Command()
                        Reef.Level.L4 -> superstructure.getPrepL4Command()
                        Reef.Level.FLOOR -> TODO()
                    },
                ),
                superstructure.getScoreCommand(),
            )

            if(index != actions.size - 1) {
                sequence.addCommands(
                    Commands.parallel(
                        AutoBuilder.followPath(getPathToSource(branch)),
                        superstructure.getStowCommand(),
                    ),
                    superstructure.getSourceIntakeCommand()
                )
            }
        }

        return sequence
    }


    private fun getPathToBranch(branch: Reef.Branch, top: Boolean): PathPlannerPath {
        return if (top) {
            PathPlannerPath.fromChoreoTrajectory("ts_${branch.name.lowercase()}")
        } else {
            PathPlannerPath.fromChoreoTrajectory("bs_${branch.name.lowercase()}")
        }
    }

    private fun getPathToSource(startBranch: Reef.Branch): PathPlannerPath {
        val topSource = startBranch in listOf(
            Reef.Branch.A,
            Reef.Branch.L,
            Reef.Branch.K,
            Reef.Branch.J,
            Reef.Branch.I,
            Reef.Branch.H
        )

        return PathPlannerPath.fromChoreoTrajectory("${startBranch.name.lowercase()}_${if (topSource) "ts" else "bs"}")

    }
}