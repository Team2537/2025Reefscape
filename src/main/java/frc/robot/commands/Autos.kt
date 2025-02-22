package frc.robot.commands

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.geometry.Pose2d
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

class Autos(
    val drivebase: Drivebase,
    val superstructure: Superstructure
) {

    private val chooser = LoggedDashboardChooser<Supplier<Command>>("auto").apply {
        addDefaultOption("ABC", { ABC_Routine.build() })
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

    val selectedRoutine: Command
        get() = chooser.get().get()

}