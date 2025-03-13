package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.MechanismVisualizer
import frc.robot.Robot
import frc.robot.Robot.drivebase
import frc.robot.subsystems.superstructure.SuperstructureGoals.ALGAE_L2
import frc.robot.subsystems.superstructure.SuperstructureGoals.ALGAE_L3
import frc.robot.subsystems.superstructure.SuperstructureGoals.L1
import frc.robot.subsystems.superstructure.SuperstructureGoals.L2
import frc.robot.subsystems.superstructure.SuperstructureGoals.L3
import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW
import frc.robot.subsystems.superstructure.arm.Arm
import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.gripper.Gripper
import frc.robot.subsystems.superstructure.manipulator.wrist.ManipulatorWrist
import lib.math.units.degrees
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class Superstructure {
    val elevator: Elevator = Elevator()
    val wrist: ManipulatorWrist = ManipulatorWrist()

    var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    fun getElevatorSysIDCommand(): Command {
        return Commands.sequence(
            elevator.getDynamicSysID(SysIdRoutine.Direction.kForward),
            elevator.getDynamicSysID(SysIdRoutine.Direction.kReverse),
            elevator.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
            elevator.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
        )
    }

    private val readyToScore: Trigger = Trigger { SmartDashboard.getBoolean("shouldScore", false) }
    
    val isL1: Trigger = Trigger { lastRequest == L1 }
    val isL2: Trigger = Trigger { lastRequest == L2 }
    val isL3: Trigger = Trigger { lastRequest == L3 }
    val isL4: Trigger = Trigger { lastRequest == L4 }

    fun getSendToStateCommand(superstructureState: Supplier<SuperstructureState>): Command {
        return Commands.sequence(
            getForceStateCommand(superstructureState),
        )
    }

    fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }

    fun getScoreCommand(): Command {
        return Commands.sequence(
            getSendToStateCommand { lastRequest },
            PrintCommand("Scoring"),
            getSendToStateCommand { STOW }
        )
    }

    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
    }
}