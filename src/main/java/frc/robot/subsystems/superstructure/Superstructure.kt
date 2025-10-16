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
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L2
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L3
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW
// import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.arm.Arm
import frc.robot.subsystems.superstructure.manipulator.Manipulator
import lib.math.units.degrees
import lib.math.units.radians
import lib.math.units.epsilonEquals
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Supplier

class Superstructure {
    val arm: Arm
    val manipulator: Manipulator

    var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    init {
        arm = Arm()
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


    fun getArmSysIDCommand(): Command {
        return Commands.sequence(
            arm.getDynamicSysID(SysIdRoutine.Direction.kForward),
            arm.getDynamicSysID(SysIdRoutine.Direction.kReverse),
            arm.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
            arm.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
        )
    }

    // fun getElevatorSysIDCommand(): Command {
    //     return Commands.sequence(
    //         elevator.getDynamicSysID(SysIdRoutine.Direction.kForward),
    //         elevator.getDynamicSysID(SysIdRoutine.Direction.kReverse),
    //         elevator.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
    //         elevator.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
    //     )
    // }

    // fun getManipulatorSysIDCommand(): Command {
    //     return Commands.sequence(
    //         manipulator.getDynamicTest(SysIdRoutine.Direction.kForward),
    //         manipulator.getDynamicTest(SysIdRoutine.Direction.kReverse),
    //         manipulator.getQuasistaticTest(SysIdRoutine.Direction.kForward),
    //         manipulator.getQuasistaticTest(SysIdRoutine.Direction.kReverse)
    //     )
    // }

    private val readyToScore: Trigger = Trigger { SmartDashboard.getBoolean("shouldScore", false) }

    val isL1: Trigger = Trigger { lastRequest == L1 }
    // val isL2: Trigger = Trigger { lastRequest == L2 }
    // val isL3: Trigger = Trigger { lastRequest == L3 }
    // val isL4: Trigger = Trigger { lastRequest == L4 }

    val isHoldingCoral: Trigger = Trigger { manipulator.inputs.coralDistance > Meters.of(0.2) }

    fun getSendToStateCommand(superstructureState: Supplier<SuperstructureState>): Command {
        return Commands.sequence(
            getForceStateCommand(superstructureState),
            arm.getMoveToAngleCommand { superstructureState.get().armAngle },
        )
    }

    fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }

    fun getStateAchievedTrigger(target: SuperstructureState): Trigger {
        return Trigger {
            Logger.recordOutput("target", target.name)
            val armAtSetpoint = arm.inputs.angle.epsilonEquals(
                target.armAngle,
                5.0.degrees
            )
            return@Trigger armAtSetpoint
        }
    }

    /// when shouldScore is true, coral will be released
    fun getScoreCommand(shouldScore: BooleanSupplier): Command {
        return Commands.sequence(
            getSendToStateCommand { lastRequest },
            Commands.waitUntil { getStateAchievedTrigger(lastRequest).asBoolean },
            Commands.waitUntil { shouldScore.getAsBoolean() },
            manipulator.getSpinRollersOutCommand(),
            Commands.waitSeconds(1.0),
            manipulator.getStopRollersCommand(),
            getSendToStateCommand { SuperstructureGoals.STOW },
            Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.STOW).asBoolean },
        )
    }

    // new intake command
    fun getIntakeCommand(): Command {
        return Commands.sequence(
            getSendToStateCommand { SuperstructureGoals.GROUND },
            Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.GROUND).asBoolean },
            manipulator.getSpinRollersInCommand(),
            Commands.waitSeconds(3.0),
            manipulator.getStopRollersCommand(),
        )
    }

    // fun getDealgaefyCommand(): Command {
    //     // when run, grab algae and return to ALGAE_STOW with manipulator rollers in brake mode
    //     return Commands.sequence(
    //         getSendToStateCommand { SuperstructureGoals.ALGAE_L2 }.onlyIf{lastRequest == SuperstructureGoals.L2},
    //         getSendToStateCommand { SuperstructureGoals.ALGAE_L3 }.onlyIf{lastRequest == SuperstructureGoals.L3},
    //         Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.ALGAE_L2).asBoolean }.onlyIf{lastRequest == SuperstructureGoals.L2},
    //         Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.ALGAE_L3).asBoolean }.onlyIf{lastRequest == SuperstructureGoals.L3},
    //         // manipulator.getSpinRollersOutCommand().until(!manipulator.isDetectingGamePiece())), // could be in?
    //         Commands.deadline( // placeholder for sim
    //             Commands.waitSeconds(3.0),
    //             manipulator.getSpinRollersOutCommand(),
    //         ),
    //         manipulator.getStopRollersCommand(),
    //         getSendToStateCommand { SuperstructureGoals.ALGAE_STOW },
    //         Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.ALGAE_STOW).asBoolean },
    //     )
    // }

    // todo, will need to release while moving up to throw algae
    // fun getReleaseAlgaeCommand(): Command {
    //     return Commands.sequence(
    //         getSendToStateCommand { lastRequest },
    //         Commands.waitUntil { getStateAchievedTrigger(lastRequest).asBoolean },
    //         manipulator.getSpinRollersInCommand(),
    //         Commands.waitSeconds(10.0),
    //         manipulator.getStopRollersCommand(),
    //         getSendToStateCommand { SuperstructureGoals.STOW },
    //         Commands.waitUntil { getStateAchievedTrigger(SuperstructureGoals.STOW).asBoolean },
    //     )
    // }

    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
        Logger.recordOutput("superstructure/setpoint/name", lastRequest.name)
    }
}