package frc.robot.commands

import choreo.Choreo.TrajectoryLogger
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.swerve.Drivebase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class Autos(
    val drivebase: Drivebase
) {
    val autoFactory: AutoFactory = AutoFactory(
        drivebase::pose,
        drivebase::resetOdometry,
        drivebase.pathFollower::accept,
        true,
        drivebase,
        { trajectory: Trajectory<SwerveSample>, isStart: Boolean ->
            if (isStart) {
                Logger.recordOutput("${drivebase.name}/traj", *trajectory.poses)
            } else {
                Logger.recordOutput("${drivebase.name}/traj", Pose2d())
            }
        }
    )

    private val chooser = LoggedDashboardChooser<AutoRoutine>("auto").apply {
        addDefaultOption("Nine Foot Test", nineFootTest())
        addOption("Two Piece", twoPiece())
        addOption("Square Test", squareTest())
        addOption("Nine Foot Test Back", nineFootTestBack())
        addOption("Nine Foot Test Left", nineFootTestLeft())
        addOption("Nine Foot Repeat", nineFootRepeat())
    }

    val selectedRoutine: AutoRoutine
        get() = chooser.get()

    fun twoPiece(): AutoRoutine {
        val routine = autoFactory.newRoutine("twoPiece")

        val startToReef = routine.trajectory("startToReef")
        val reefToSource = routine.trajectory("reefToSource")
        val sourceToReef = routine.trajectory("sourceToReef")

        routine.active()
            .onTrue(
                Commands.sequence(
                    startToReef.resetOdometry(),
                    startToReef.cmd(),
                    drivebase.getStopCmd(),
                    Commands.waitSeconds(3.0),
                    reefToSource.cmd(),
                    drivebase.getStopCmd(),
                    Commands.waitSeconds(3.0),
                    sourceToReef.cmd(),
                    drivebase.getStopCmd()
                )
            )

        return routine

    }

    fun nineFootTest(): AutoRoutine {
        val routine = autoFactory.newRoutine("nineFootTest")

        val nineFootForwards = routine.trajectory("9ftForward")
        val nineFootBack = routine.trajectory("9ftBack")

        routine.active().onTrue(
            Commands.sequence(
                nineFootForwards.resetOdometry(),
                nineFootForwards.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(3.0),
                nineFootBack.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }

    fun nineFootRepeat(): AutoRoutine {
        val routine = autoFactory.newRoutine("nineFootRepeat")

        val nineFootForwards = routine.trajectory("9ftForward")
        val nineFootBack = routine.trajectory("9ftBack")

        routine.active().onTrue(
            Commands.sequence(
                nineFootForwards.resetOdometry(),
                nineFootForwards.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                nineFootBack.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                nineFootForwards.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                nineFootBack.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                nineFootForwards.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(1.0),
                nineFootBack.cmd(),
                drivebase.getStopCmd(),
            )
        )

        return routine
    }

    fun nineFootTestLeft(): AutoRoutine {
        val routine = autoFactory.newRoutine("nineFootTestLeft")

        val nineFootLeft = routine.trajectory("9ftLeft")
        val nineFootRight = routine.trajectory("9ftRight")

        routine.active().onTrue(
            Commands.sequence(
                nineFootLeft.resetOdometry(),
                nineFootLeft.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(3.0),
                nineFootRight.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }

    fun nineFootTestBack(): AutoRoutine {
        val routine = autoFactory.newRoutine("nineFootTestBack")

        val nineFootBack = routine.trajectory("9ftBack")
        val nineFootForwards = routine.trajectory("9ftForward")

        routine.active().onTrue(
            Commands.sequence(
                nineFootBack.resetOdometry(),
                nineFootBack.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(3.0),
                nineFootForwards.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }

    fun squareTest(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("squareTest")

        val square = routine.trajectory("squareTest")

        routine.active().onTrue(
            Commands.sequence(
                square.resetOdometry(),
                square.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }

    fun wiggly(): AutoRoutine {
        val routine = autoFactory.newRoutine("wiggly")

        val wiggly = routine.trajectory("wiggly")

        routine.active().onTrue(
            Commands.sequence(
                wiggly.resetOdometry(),
                wiggly.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }
}