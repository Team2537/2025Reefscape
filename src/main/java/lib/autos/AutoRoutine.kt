package lib.autos

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.swerve.Drivebase
import lib.math.geometry.FieldConstants.Reef

class AutoRoutine(
    private val actions: List<Pair<Reef.Branch, Reef.Level>>,
    private val drivebase: Drivebase,
    private val superstructure: Superstructure
) {
    
    fun build(): Command {
        val sequence = SequentialCommandGroup()
        
        actions.forEachIndexed { index, (branch, level) ->
            sequence.addCommands(
                Commands.parallel(
                    getPathToBranch(branch),
                    when (level) {
                        Reef.Level.L1 -> superstructure.getPrepL1Command()
                        Reef.Level.L2 -> superstructure.getPrepL2Command()
                        Reef.Level.L3 -> superstructure.getPrepL3Command()
                        Reef.Level.L4 -> superstructure.getPrepL4Command()
                    },
                ),
                superstructure.getScoreCommand(),
                getPathToSource(branch)
            )
        }
        
        return sequence
    }
    
    
    private fun getPathToBranch(branch: Reef.Branch): Command {
        val pose = drivebase.pose.translation
        if(pose.y > 4.00) {
            val path = PathPlannerPath.fromChoreoTrajectory("TOP_SOURCE_TO_${branch.name}")
            return AutoBuilder.followPath(path)
        } else {
            val path = PathPlannerPath.fromChoreoTrajectory("BOTTOM_SOURCE_TO_${branch.name}")
            return AutoBuilder.followPath(path)
        }
    }
    
    private fun getPathToSource(startBranch: Reef.Branch): Command {
        val topSource = startBranch in listOf(
            Reef.Branch.A,
            Reef.Branch.L,
            Reef.Branch.K,
            Reef.Branch.J,
            Reef.Branch.I,
            Reef.Branch.H
        )
        
        val path =
            PathPlannerPath.fromChoreoTrajectory("${startBranch.name}_TO_${if (topSource) "TOP" else "BOTTOM"}_SOURCE")
        
        return AutoBuilder.followPath(path)
    }
}