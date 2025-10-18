package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.swerve.AlignmentCommand
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructureGoals
import frc.robot.subsystems.swerve.Drivebase
import lib.autos.AutoRoutine
import lib.math.geometry.FieldConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class Autos(
    val drivebase: Drivebase,
    val superstructure: Superstructure,
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
        // addDefaultOption("IDLE", { Commands.sequence(superstructure.getSendToStateCommand { SuperstructureGoals.STOW }, Commands.idle() )})
        addOption("IJ", { IJ_Routine.build() })
        addOption("I4", { I4_Routine.build() })
        addDefaultOption("DRIVE_FORWARDS", {driveForwards()})
//        addOption("FE", { FE_Routine.build() })
//        addOption("F4", { F4_Routine.build() })
//        addOption("B - L2, L3", { bL2_L3() })
//        addOption("idle", { idle_Routine.build()})
    }

    fun driveForwards(): Command {
        return Commands.sequence(
            Commands.runOnce({
                drivebase.applyChassisSpeeds(ChassisSpeeds(-3.0, 0.0, 0.0))
            }, drivebase),
            Commands.waitSeconds(5.0),
            drivebase.getStopCmd()
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
        superstructure,
    )

    val IJ_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
            Triple(FieldConstants.Reef.Branch.J, FieldConstants.Reef.Level.L4, true),
        ),
        drivebase,
        superstructure,

        )

    val I4_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
        ),
        drivebase,
        superstructure,

        )

    val FE_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true),
            Triple(FieldConstants.Reef.Branch.E, FieldConstants.Reef.Level.L4, true),
        ),
        drivebase,
        superstructure,

        )

    val F4_Routine: AutoRoutine = AutoRoutine(
        listOf(
            Triple(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true),
        ),
        drivebase,
        superstructure,

        )

    val idle_Routine: AutoRoutine = AutoRoutine(
        listOf(
//            Triple(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
//            Triple(FieldConstants.Reef.Branch.J, FieldConstants.Reef.Level.L4, true),
        ),
        drivebase,
        superstructure,

        )

    val selectedRoutine: Command
        get() = chooser.get().get()

}