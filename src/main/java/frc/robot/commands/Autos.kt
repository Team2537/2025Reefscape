package frc.robot.commands

import choreo.auto.AutoRoutine
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.swerve.Drivebase
import lib.math.geometry.FieldConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier

class Autos(
    val drivebase: Drivebase,
    val superstructure: Superstructure
) {

    private val chooser = LoggedDashboardChooser<AutoRoutine>("auto").apply {
        addDefaultOption("IJ", IJ_Routine())
    }


    fun IJ_Routine(): AutoRoutine {
        val loop = drivebase.autoFactory.newRoutine("IJ_Routine")

        val startToI = loop.trajectory("top_start_i")
        val iToSource = loop.trajectory("i_ts")
        val sourceToJ = loop.trajectory("ts_j")

        loop.active().onTrue(
            Commands.sequence(
                startToI.resetOdometry(),
                startToI.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                iToSource.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                sourceToJ.cmd(),
                drivebase.getStopCmd()
            )
        )

        return loop
    }


    val selectedRoutine: AutoRoutine
        get() = chooser.get()

}