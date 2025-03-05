package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.swerve.Drivebase
import lib.autos.AutoRoutine
import lib.math.geometry.FieldConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class Autos(
    val drivebase: Drivebase,
    val superstructure: Superstructure
) {

    val IJ_Routine_PP = Supplier {
        val pp_topstart_i = PathPlannerPath.fromPathFile("pp_topstart_i")

        Commands.sequence(
            AutoBuilder.resetOdom(pp_topstart_i.startingHolonomicPose.getOrDefault(Pose2d())),
            AutoBuilder.followPath(pp_topstart_i),
            drivebase.getStopCmd(),
            Commands.waitSeconds(1.0),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("i_topsource")),
            drivebase.getStopCmd(),
            Commands.waitSeconds(1.0),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("topsource_j")),
            drivebase.getStopCmd(),
        )
    }

    private val chooser = LoggedDashboardChooser<Supplier<Command>>("auto").apply {
//        addDefaultOption("IJ", {IJ_Routine.build()})
        addDefaultOption("B - L2, L3", { bL2_L3() })
    }

    private fun bL2_L3(): Command {
        val b_to_source = PathPlannerPath.fromPathFile("B_to_bs")
        val source_to_b = PathPlannerPath.fromPathFile("bs_to_B")

        return Commands.sequence(
            AutoBuilder.resetOdom(b_to_source.startingHolonomicPose.getOrDefault(Pose2d())),
            superstructure.getPrepL2Command(),
            superstructure.getWaitUntilAtPositionCmd(),
            superstructure.getScoreCommand(),
            Commands.parallel(
                AutoBuilder.followPath(b_to_source),
                Commands.sequence(
                    Commands.waitSeconds(0.15),
                    superstructure.getSourceIntakeCommand()
                )
            ),
            Commands.parallel(
                AutoBuilder.followPath(source_to_b),
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    superstructure.getStowCommand()
                )
            ),
            superstructure.getPrepL3Command(),
            superstructure.getWaitUntilAtPositionCmd(),
            superstructure.getScoreCommand()
        )
    }

    val ABC_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L4, true),
            Triple(FieldConstants.Reef.Branch.B, FieldConstants.Reef.Level.L2, true),
            Triple(FieldConstants.Reef.Branch.C, FieldConstants.Reef.Level.L3, false),
            Triple(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L3, false),
        ),
        drivebase,
        superstructure
    )

    val IJ_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
            Triple(FieldConstants.Reef.Branch.J, FieldConstants.Reef.Level.L2, true),
        ),
        drivebase,
        superstructure
    )


    val selectedRoutine: Command
        get() = chooser.get().get()

}