package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.CoralSimulator
import frc.robot.MechanismVisualizer
import frc.robot.Robot
import frc.robot.Robot.drivebase
import frc.robot.subsystems.superstructure.arm.Arm
import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.gripper.Gripper
import lib.math.units.degrees
import lib.math.units.inches
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class Superstructure {
    val elevator: Elevator = Elevator()
    val arm: Arm = Arm()
    val gripper: Gripper = Gripper()

    private var lastRequest: SuperstructureState = SuperstructureGoals.STOW

    val gamepieceSimulator = CoralSimulator(
        { MechanismVisualizer.mechanismPoses[2] },
        drivebase::pose,
        Inches.of(24.0)
    )

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
        ) { lastRequest.nextState.isPresent }
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

    fun getStowCommand(): Command {
        return Commands.sequence(
            getForceStateCommand { SuperstructureGoals.STOW },
            Commands.parallel(
                Commands.print("Stowing"),
                elevator.getMoveToHeightCommand { lastRequest.elevatorHeight },
                arm.getSendToAngleCmd { lastRequest.armAngle },
            ),
        )
    }

    fun getSourceIntakeCommand(): Command {
        return Commands.sequence(
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
            gripper.getIntakeCmd(),
            getStowCommand()
        )
    }

    private fun getForceStateCommand(stateSupplier: Supplier<SuperstructureState>): Command {
        return runOnce({
            lastRequest = stateSupplier.get()
            drivebase.limits = lastRequest.driveLimits
        })
    }


    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest)
        gamepieceSimulator.update()
    }
}