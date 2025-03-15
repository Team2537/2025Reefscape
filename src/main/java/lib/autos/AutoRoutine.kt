package lib.autos

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.swerve.AlignmentCommand
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructureGoals.L1
import frc.robot.subsystems.superstructure.SuperstructureGoals.L2
import frc.robot.subsystems.superstructure.SuperstructureGoals.L3
import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW
import frc.robot.subsystems.swerve.Drivebase
import lib.math.geometry.FieldConstants.Reef
import lib.math.geometry.flipped
import lib.math.units.degrees
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class AutoRoutine(
    private val actions: List<Triple<Reef.Branch, Reef.Level, Boolean>>,
    private val drivebase: Drivebase,
    private val superstructure: Superstructure,
) {
    
    fun build(): Command {
        val sequence = SequentialCommandGroup()

        val startPath = getPathFromStart(actions.first().first)

        sequence.addCommands(
            Commands.sequence(
                AutoBuilder.resetOdom(startPath.startingHolonomicPose.getOrDefault(Pose2d())),
                superstructure.getSendToStateCommand { STOW },
                drivebase.followPath(startPath),
            )
        )

        actions.forEachIndexed { index, (branch, level, isTop) ->
            sequence.addCommands(
                Commands.parallel(
                    drivebase.followPath(getPathToBranch(branch, isTop)).onlyIf({ index != 0 }),
                    when (level) {
                        Reef.Level.L1 -> superstructure.getForceStateCommand { L1 }
                        Reef.Level.L2 -> superstructure.getForceStateCommand { L2 }
                        Reef.Level.L3 -> superstructure.getForceStateCommand { L3 }
                        Reef.Level.L4 -> superstructure.getForceStateCommand { L4 }
                        else -> superstructure.getForceStateCommand { STOW }
                    }
                ),
                drivebase.getStopCmd(),
                Commands.waitSeconds(0.75),
                superstructure.getScoreCommand(),
            )

            if (index != actions.size - 1) {
                sequence.addCommands(
                    Commands.parallel(
                        drivebase.followPath(getPathToSource(branch)),
                        superstructure.getSendToStateCommand({ STOW }),
                    ),
                    drivebase.getStopCmd(),
                    Commands.waitSeconds(2.0),
                )
            }
        }

        return sequence
    }

    
    private fun getPathToBranch(branch: Reef.Branch, top: Boolean): PathPlannerPath {
        return if (top) {
            PathPlannerPath.fromPathFile("ts_to_${branch.name}")
        } else {
            PathPlannerPath.fromPathFile("bs_to_${branch.name}")
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
        
        return PathPlannerPath.fromPathFile("${startBranch.name}_to_${if (topSource) "ts" else "bs"}")
        
    }
    
    fun getPathFromStart(branch: Reef.Branch): PathPlannerPath {
        val topBranches = listOf(
            Reef.Branch.J,
            Reef.Branch.I,
        )
        
        return if (topBranches.contains(branch)) {
            PathPlannerPath.fromPathFile("tstart_to_${branch.name}")
        } else {
            PathPlannerPath.fromPathFile("bstart_to_${branch.name}")
        }
    }
}