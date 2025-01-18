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

class Autos(
    val drivebase: Drivebase
) {

    fun getAutoCommand(): Command {
        return tenFootTest().cmd()
    }

    val autoFactory: AutoFactory = AutoFactory(
        drivebase::pose,
        drivebase::resetOdometry,
        drivebase.pathFollower::accept,
        true,
        drivebase,
        { trajectory: Trajectory<SwerveSample>, isStart: Boolean ->
            if(isStart) {
                Logger.recordOutput("${drivebase.name}/traj", *trajectory.poses)
            } else {
                Logger.recordOutput("${drivebase.name}/traj", Pose2d())
            }
        }
    )

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

    fun tenFootTest(): AutoRoutine {
        val routine = autoFactory.newRoutine("tenFootTest")

        val tenFootForwards = routine.trajectory("10ftForward")
        val tenFootBack = routine.trajectory("10ftBack")

        routine.active().onTrue(
            Commands.sequence(
                tenFootForwards.resetOdometry(),
                tenFootForwards.cmd(),
                drivebase.getStopCmd(),
                Commands.waitSeconds(3.0),
                tenFootBack.cmd(),
                drivebase.getStopCmd()
            )
        )

        return routine
    }
}