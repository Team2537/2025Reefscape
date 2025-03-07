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
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW
import frc.robot.subsystems.superstructure.arm.Arm
import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.gripper.Gripper
import lib.math.units.degrees
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class Superstructure {
    val elevator: Elevator = Elevator()
    val arm: Arm = Arm()
    val gripper: Gripper = Gripper()

    private var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    val coralPositionSupplier: Supplier<Distance> = Supplier { gripper.inputs.coralDistance }

    fun getArmSysIDCommand(): Command {
        return Commands.sequence(
            arm.getDynamicTest(SysIdRoutine.Direction.kReverse),
            arm.getDynamicTest(SysIdRoutine.Direction.kForward),
            arm.getQuasistaticTest(SysIdRoutine.Direction.kReverse),
            arm.getQuasistaticTest(SysIdRoutine.Direction.kForward)
        )
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

    fun getScoreCommand(): Command {
        return Commands.either(
            Commands.sequence(
                getForceStateCommand { lastRequest.nextState.get() },
                Commands.parallel(
                    elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                    arm.getSendToAngleCmd { lastRequest.armAngle },
                ),
                Commands.waitUntil(arm.getAngleInToleranceTrigger(1.0.degrees)),
                gripper.getEjectCmd(),
                getStowCommand()
            ),
            Commands.idle()
        ) { lastRequest.nextState.isPresent }.handleInterrupt { gripper.io.setVoltage(Volts.zero()) }
    }

    fun getPrepL1Command(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.L1_PREP },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getPrepL2Command(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.L2_PREP },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getPrepL3Command(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.L3_PREP },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getPrepL4Command(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.L4_PREP },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getPrepL2DealgaefyCmd(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.PREP_ALGAE_L2 },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getPrepL3DealgaefyCmd(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.PREP_ALGAE_L3 },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getStowCommand(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.STOW },
            Commands.sequence(
                arm.getSendToAngleCmd { lastRequest.armAngle },
                Commands.waitUntil { arm.inputs.motorRelativePosition > 65.0.degrees },
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight }
            )
        )
    }

    fun getSourceIntakeCommand(): Command {
        return Commands.sequence(
            Commands.sequence(
                getForceStateCommand { SuperstructureGoals.PRE_SOURCE },
                Commands.parallel(
                    elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                    arm.getSendToAngleCmd { lastRequest.armAngle },
                ),
                Commands.waitUntil(
                    elevator.getPositionInToleranceTrigger(6.0.inches)
                        .and(arm.getAngleInToleranceTrigger(5.0.degrees))
                ),
                getForceStateCommand({ SuperstructureGoals.SOURCE }),
                Commands.parallel(
                    elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                    arm.getSendToAngleCmd { lastRequest.armAngle },
                ),
            ).onlyIf({ lastRequest != SuperstructureGoals.SOURCE }),
            gripper.getIntakeCmd(),
        )
    }

    fun getDealgaefyCommand(): Command {
        return Commands.sequence(
            getForceStateCommand { lastRequest.nextState.get() },
            arm.getSendToAngleCmd({ lastRequest.armAngle }),
            Commands.waitUntil(arm.getAngleInToleranceTrigger(1.0.degrees)),
            Commands.parallel(
                elevator.getMoveToHeightCommand({ lastRequest.elevatorHeight }),
                gripper.getDealgaefyCmd(),
            ),
            getStowCommand()
        ).onlyIf {
            lastRequest.nextState.isPresent
                    && lastRequest.nextState.getOrDefault(STOW) in listOf(ALGAE_L2, ALGAE_L3)
        }.handleInterrupt { gripper.io.setVoltage(Volts.zero()) }
    }

    fun getClimbCommand(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.PRE_CLIMB },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
            getWaitUntilAtPositionCmd(),
            getForceStateCommand { SuperstructureGoals.CLIMB },
            Commands.parallel(
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
            getWaitUntilAtPositionCmd()
        )
    }

    fun getWaitUntilAtPositionCmd(): Command = Commands.waitUntil(
        elevator.getPositionInToleranceTrigger(2.0.inches)
            .and(arm.getAngleInToleranceTrigger(5.0.degrees))
    )

    private fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }


    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
    }
}