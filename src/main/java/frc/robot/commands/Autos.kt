package frc.robot.commands

import choreo.Choreo.TrajectoryLogger
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.swerve.Drivebase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier

class Autos(
    val drivebase: Drivebase
) {

    private val chooser = LoggedDashboardChooser<Command>("auto").apply {
        addDefaultOption("Nine Foot Test", nineFootTest())
    }

    fun nineFootTest(): Command {
        return PathPlannerAuto("9ftTest")
    }

    fun nineFootTestChoreo(): Command {
        return PathPlannerAuto("9ftTestChoreo")
    }

    val selectedRoutine: Command
        get() = chooser.get()

}