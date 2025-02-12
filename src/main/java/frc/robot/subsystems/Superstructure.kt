package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import lib.math.units.degrees
import lib.math.units.meters
import lib.math.units.radians
import org.littletonrobotics.junction.Logger
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
    
    fun getNextReefNodeCmd(): Command {
        return Commands.sequence(
            runOnce({
                val levels =
                    SuperstructureSetpoint.entries.filter { it.toString().contains(Regex("L[0-9]+")) }.sortedBy { it.elevatorHeight }
                
                setpoint = when (setpoint) {
                    SuperstructureSetpoint.L4 -> return@runOnce
                    in levels -> levels[levels.indexOf(setpoint) + 1]
                    else -> levels.first()
                }
            }),
            Commands.parallel(
                Commands.defer({elevator.getSendToNodeCmd(setpoint)}, setOf(elevator)),
                Commands.defer({arm.getSendToAngleCmd(setpoint.armAngle)}, setOf(arm))
            )
        )
    }
    
    fun getPreviousReefNodeCmd(): Command {
        return Commands.sequence(
            runOnce({
                val levels =
                    SuperstructureSetpoint.entries.filter { it.toString().contains(Regex("L[0-9]+")) }.sortedBy { it.elevatorHeight }
                
                setpoint = when (setpoint) {
                    SuperstructureSetpoint.L1 -> return@runOnce
                    in levels -> levels[levels.indexOf(setpoint) - 1]
                    else -> levels.last()
                }
            }),
            Commands.parallel(
                Commands.defer({elevator.getSendToNodeCmd(setpoint)}, setOf(elevator)),
                Commands.defer({arm.getSendToAngleCmd(setpoint.armAngle)}, setOf(arm))
            )
        )
    }
    
    fun getSendToHomeCmd(): Command {
        return Commands.parallel(
            runOnce({ setpoint = SuperstructureSetpoint.HOME }),
            elevator.getSendToNodeCmd(SuperstructureSetpoint.HOME),
            arm.getSendToAngleCmd(SuperstructureSetpoint.HOME.armAngle)
        )
    }
    
    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", setpoint)
    }
}