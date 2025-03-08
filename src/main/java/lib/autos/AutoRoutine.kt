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
    private val climb: Climb
) {
    
    fun build(): Command {
        val sequence = SequentialCommandGroup()

        if (actions.isEmpty()) {
            sequence.addCommands(
                Commands.sequence(
                    climb.getVoltageControlCommand(Supplier<Double> { 12.0 }).withTimeout(2.5),
                    climb.getVoltageControlCommand(Supplier<Double> {0.0}),
                    superstructure.getStowCommand(),
                    AutoBuilder.resetOdom(Pose2d())

                )
            )
            return sequence
        }

        val startPath = getPathFromStart(actions.first().first)
        
        sequence.addCommands(
            Commands.sequence(
                climb.getVoltageControlCommand(Supplier<Double> { 12.0 }).withTimeout(2.5),
                climb.getVoltageControlCommand(Supplier<Double> {0.0}).withTimeout(0.5),
                superstructure.getStowCommand(),
                AutoBuilder.resetOdom(startPath.startingHolonomicPose.getOrDefault(Pose2d())),
                drivebase.followPath(startPath),
            )
        )
        
        actions.forEachIndexed { index, (branch, level, isTop) ->
            sequence.addCommands(
                Commands.parallel(
                    drivebase.followPath(getPathToBranch(branch, isTop)).onlyIf({ index != 0 }),
                    when (level) {
                        Reef.Level.L1 -> superstructure.getPrepL1Command()
                        Reef.Level.L2 -> superstructure.getPrepL2Command()
                        Reef.Level.L3 -> superstructure.getPrepL3Command()
                        Reef.Level.L4 -> superstructure.getPrepL4Command()
                        Reef.Level.FLOOR -> TODO()
                    }
                ),
                PrintCommand("here"),
                AlignmentCommand.nodeAlignment(
                    drivebase,
                    if (branch in listOf(
                            Reef.Branch.A,
                            Reef.Branch.C,
                            Reef.Branch.E,
                            Reef.Branch.G,
                            Reef.Branch.I,
                            Reef.Branch.K
                        )
                    ) Reef.Side.LEFT else Reef.Side.RIGHT, superstructure.coralPositionSupplier, { level == Reef.Level.L4}
                ),
                Commands.waitSeconds(0.75),
                superstructure.getScoreCommand(),
            )
            
            if (index != actions.size - 1) {
                sequence.addCommands(
                    Commands.parallel(
                        drivebase.followPath(getPathToSource(branch)),
                        superstructure.getStowCommand(),
                    ),
                    drivebase.getStopCmd(),
                    superstructure.getSourceIntakeCommand(),
                    Commands.waitSeconds(0.75)
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