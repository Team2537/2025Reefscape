package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Robot.drivebase
import frc.robot.subsystems.superstructure.SuperstructureGoals.L1
import frc.robot.subsystems.superstructure.SuperstructureGoals.L2
import frc.robot.subsystems.superstructure.SuperstructureGoals.L3
import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW
import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.manipulator.Manipulator
import lib.math.units.degrees
import lib.math.units.epsilonEquals
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Supplier

class Superstructure {
    val elevator: Elevator
    val manipulator: Manipulator

    var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    init {
        elevator = Elevator()
        manipulator = Manipulator()

        // manipulator.apply {
        //     defaultCommand = this.run {
        //         if (inputs.coralDistance > Meters.of(0.2)) {
        //             io.setRollerVoltage(Volts.of(1.0))
        //         } else {
        //             io.setRollerVoltage(Volts.zero())
        //         }
        //     }.onlyIf { lastRequest == STOW }.handleInterrupt { io.setRollerVoltage(Volts.zero()) }
        // }

        // elevator.apply {
        //     defaultCommand = this.run {
        //         if(lastRequest == STOW && manipulator.inputs.coralDistance < Meters.of(0.2)) {
        //             io.setElevatorHeightTarget(STOW.elevatorHeight + 12.0.inches)
        //         } else {
        //             io.setElevatorHeightTarget(STOW.elevatorHeight)
        //         }
        //     }
        // }
    }


    fun getElevatorSysIDCommand(): Command {
        return Commands.sequence(
            elevator.getDynamicSysID(SysIdRoutine.Direction.kForward),
            elevator.getDynamicSysID(SysIdRoutine.Direction.kReverse),
            elevator.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
            elevator.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
        )
    }

    fun getManipulatorSysIDCommand(): Command {
        return Commands.sequence(
            manipulator.getDynamicTest(SysIdRoutine.Direction.kForward),
            manipulator.getDynamicTest(SysIdRoutine.Direction.kReverse),
            manipulator.getQuasistaticTest(SysIdRoutine.Direction.kForward),
            manipulator.getQuasistaticTest(SysIdRoutine.Direction.kReverse)
        )
    }

    private val readyToScore: Trigger = Trigger { SmartDashboard.getBoolean("shouldScore", false) }

    val isL1: Trigger = Trigger { lastRequest == L1 }
    val isL2: Trigger = Trigger { lastRequest == L2 }
    val isL3: Trigger = Trigger { lastRequest == L3 }
    val isL4: Trigger = Trigger { lastRequest == L4 }

    val isHoldingCoral: Trigger = Trigger { manipulator.inputs.coralDistance > Meters.of(0.2) }

    fun getSendToStateCommand(superstructureState: Supplier<SuperstructureState>): Command {
        return Commands.sequence(
            getForceStateCommand(superstructureState),
            Commands.parallel(
                elevator.getMoveToHeightCommand { superstructureState.get().elevatorHeight },
                manipulator.getSendToAngleCommand(superstructureState.get().armAngle)
            )
        )
    }

    fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }

    fun getScoreCommand(shouldScore: BooleanSupplier): Command {
        return Commands.waitSeconds(0.5) // do nothing for now
    }

    fun getDealgaefyCommand(): Command {
        return Commands.waitSeconds(0.5) // do nothing for now
    }

    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
        Logger.recordOutput("superstructure/setpoint/name", lastRequest.name)
    }
}