package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import lib.commands.not
import lib.math.units.degrees
import lib.math.units.meters
import lib.math.units.radians
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Supplier

class Superstructure {
    val elevator: Elevator = Elevator()
    val arm: Arm = Arm()

    private var setpoint: SuperstructureSetpoint = SuperstructureSetpoint.HOME

    enum class SuperstructureSetpoint(val elevatorHeight: Distance, val armAngle: Angle) {
        HOME(0.07.meters, 90.0.degrees),
        SOURCE(0.996.meters, (-2.461).radians),
        L1(0.028.meters, 0.862.radians),
        L2(0.238.meters, 0.862.radians),
        L3(0.642.meters, 0.862.radians),
        L4(1.289.meters, 0.862.radians),
    }

    private val setpointSupplier: Supplier<SuperstructureSetpoint> = Supplier { setpoint }

    private val reefLevels = listOf(
        SuperstructureSetpoint.L1,
        SuperstructureSetpoint.L2,
        SuperstructureSetpoint.L3,
        SuperstructureSetpoint.L4
    )

    private val readyToScore: Trigger = Trigger { SmartDashboard.getBoolean("shouldScore", false) }

    fun getNextReefNodeCmd(): Command {
        return Commands.sequence(
            runOnce({
                setpoint = when (setpoint) {
                    SuperstructureSetpoint.L4 -> return@runOnce
                    in reefLevels -> reefLevels[reefLevels.indexOf(setpoint) + 1]
                    else -> reefLevels.first()
                }
            }),
            Commands.parallel(
                Commands.defer({ elevator.getSendToNodeCmd(setpoint) }, setOf(elevator)),
                Commands.defer({ arm.getSendToNodeCmd(setpoint) }, setOf(arm))
            )
        )
    }

    fun getPreviousReefNodeCmd(): Command {
        return Commands.sequence(
            runOnce({
                setpoint = when (setpoint) {
                    SuperstructureSetpoint.L1 -> return@runOnce
                    in reefLevels -> reefLevels[reefLevels.indexOf(setpoint) - 1]
                    else -> reefLevels.last()
                }
            }),
            Commands.parallel(
                Commands.defer({ elevator.getSendToNodeCmd(setpoint) }, setOf(elevator)),
                Commands.defer({ arm.getSendToNodeCmd(setpoint) }, setOf(arm))
            )
        )
    }

    fun getSendToHomeCmd(): Command {
        return Commands.parallel(
            runOnce({ setpoint = SuperstructureSetpoint.HOME }),
            elevator.getSendToNodeCmd(SuperstructureSetpoint.HOME),
            arm.getSendToNodeCmd(SuperstructureSetpoint.HOME)
        )
    }

    fun getScoreNodeCmd(
        cancelOverride: Trigger,
        scoreTrigger: Trigger,
        isHoldingCoral: BooleanSupplier,
        node: SuperstructureSetpoint
    ): Command {
        if (node == SuperstructureSetpoint.HOME || node == SuperstructureSetpoint.SOURCE) {
            throw IllegalArgumentException("Can only score at L1-L4")
        }

        return Commands.sequence(
            Commands.parallel(
                runOnce({ setpoint = node }),
                elevator.getSendToNodeCmd(node),
                arm.getSendToNodeCmd(node),
            ),
            Commands.waitUntil(scoreTrigger.or(cancelOverride)),
            Commands.either(
                Commands.sequence(
                    Commands.print("Outtaking"),
                    Commands.waitUntil(isHoldingCoral),
                    getSendToHomeCmd()
                ),
                getSendToHomeCmd(),
                scoreTrigger.and(!cancelOverride)
            )
        )
    }

    fun getSendToSourceCmd(): Command {
        return Commands.parallel(
            runOnce({ setpoint = SuperstructureSetpoint.SOURCE }),
            elevator.getSendToNodeCmd(SuperstructureSetpoint.SOURCE),
            arm.getSendToNodeCmd(SuperstructureSetpoint.SOURCE)
        )
    }

    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", setpoint)
    }
}