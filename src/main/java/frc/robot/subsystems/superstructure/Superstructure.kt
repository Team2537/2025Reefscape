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
import frc.robot.subsystems.superstructure.manipulator.roller.ManipulatorRoller
import frc.robot.subsystems.superstructure.manipulator.wrist.ManipulatorWrist
import lib.math.units.degrees
import lib.math.units.epsilonEquals
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Supplier

class Superstructure {
    val elevator: Elevator
    val wrist: ManipulatorWrist
    val rollers: ManipulatorRoller

    var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    init {
        elevator = Elevator()
        wrist = ManipulatorWrist()
        rollers = ManipulatorRoller()

        rollers.apply {
            defaultCommand = this.run {
                if (inputs.coralDistance > 0.2) {
                    io.setVoltage(Volts.of(1.0))
                } else {
                    io.setVoltage(Volts.zero())
                }
            }.onlyIf { lastRequest == STOW }.handleInterrupt { io.setVoltage(Volts.zero()) }
        }

        elevator.apply {
            defaultCommand = this.run {
                if(lastRequest == STOW && rollers.inputs.coralDistance < 0.2) {
                    io.setElevatorHeightTarget(STOW.elevatorHeight + 12.0.inches)
                } else {
                    io.setElevatorHeightTarget(STOW.elevatorHeight)
                }
            }
        }
    }


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

    val isHoldingCoral: Trigger = Trigger { rollers.inputs.coralDistance > 0.2 }

    fun getSendToStateCommand(superstructureState: Supplier<SuperstructureState>): Command {
        return Commands.sequence(
            getForceStateCommand(superstructureState),
            Commands.parallel(
                elevator.getMoveToHeightCommand { superstructureState.get().elevatorHeight },
                wrist.getSendToAngleCmd { superstructureState.get().armAngle }
            )
        )
    }

    fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }

    fun getDealgaefyCommand(): Command {
        return Commands.sequence(
            elevator.getMoveToHeightCommand { Inches.of(9.0) },
            Commands.waitUntil(elevator.getPositionInToleranceTrigger(0.5.inches)),
            wrist.getSendToAngleCmd { Degrees.of(140.0) },
            Commands.waitUntil { wrist.inputs.angle.epsilonEquals(140.0.degrees, 5.0.degrees) },
            Commands.parallel(
                wrist.getSendToAngleCmd { Degrees.of(170.0) },
                elevator.getMoveToHeightCommand {
                    if (lastRequest == L3) Inches.of(29.0)
                    else Inches.of(12.0)
                },
                rollers.getDealgaefyCommand()
            ).onlyIf { lastRequest == L3 || lastRequest == L2 },

            )
    }


    fun getScoreCommand(readyToScore: BooleanSupplier): Command {
        return Commands.sequence(
            Commands.either(
                Commands.sequence(
                    elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                    Commands.waitUntil(
                        elevator.getPositionInToleranceTrigger(Inches.of(3.0))
                            .and { elevator.inputs.carriageHeight > 3.0.inches }),
                    wrist.getSendToAngleCmd { lastRequest.armAngle },
                    Commands.waitUntil { wrist.inputs.angle.epsilonEquals(lastRequest.armAngle, Degrees.of(5.0)) },
                    Commands.waitSeconds(0.4)
                ),
                Commands.sequence(
                    getSendToStateCommand { lastRequest },
                    Commands.waitUntil(
                        elevator.getPositionInToleranceTrigger(Inches.of(0.5))
                            .and { wrist.inputs.angle.epsilonEquals(lastRequest.armAngle, Degrees.of(5.0)) }
                    ),
                ),
                { lastRequest == L4 }
            ),
            Commands.waitUntil(readyToScore),
            rollers.getScoreCommand(),
            getSendToStateCommand { STOW }
        ).onlyIf { lastRequest != STOW } // if last request was stow, return command that does nothing
    }

    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
        Logger.recordOutput("superstructure/setpoint/name", lastRequest.name)
    }
}