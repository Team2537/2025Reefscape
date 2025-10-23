package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.SuperstructureGoals.L1;
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L2;
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L3;
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L4;
import frc.robot.subsystems.superstructure.SuperstructureGoals.STOW;
// import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import lib.math.units.degrees;
import lib.math.units.radians;
import lib.math.units.epsilonEquals;
import lib.math.units.inches;
import org.littletonrobotics.junction.Logger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure {
    public final Arm arm;
    public final Manipulator manipulator;

    public SuperstructureState lastRequest = SuperstructureGoals.STOW;

    public Superstructure() {
        arm = new Arm();
        manipulator = new Manipulator();

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


    public Command getArmSysIDCommand() {
        return Commands.sequence(
            arm.getDynamicSysID(SysIdRoutine.Direction.kForward),
            arm.getDynamicSysID(SysIdRoutine.Direction.kReverse),
            arm.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
            arm.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
        );
    }

    // public Command getElevatorSysIDCommand() {
    //     return Commands.sequence(
    //         elevator.getDynamicSysID(SysIdRoutine.Direction.kForward),
    //         elevator.getDynamicSysID(SysIdRoutine.Direction.kReverse),
    //         elevator.getQuasistaticSysID(SysIdRoutine.Direction.kForward),
    //         elevator.getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
    //     );
    // }

    // public Command getManipulatorSysIDCommand() {
    //     return Commands.sequence(
    //         manipulator.getDynamicTest(SysIdRoutine.Direction.kForward),
    //         manipulator.getDynamicTest(SysIdRoutine.Direction.kReverse),
    //         manipulator.getQuasistaticTest(SysIdRoutine.Direction.kForward),
    //         manipulator.getQuasistaticTest(SysIdRoutine.Direction.kReverse)
    //     );
    // }

    private final Trigger readyToScore = new Trigger(() -> SmartDashboard.getBoolean("shouldScore", false));

    public final Trigger isL1 = new Trigger(() -> lastRequest == L1);
    // public final Trigger isL2 = new Trigger(() -> lastRequest == L2);
    // public final Trigger isL3 = new Trigger(() -> lastRequest == L3);
    // public final Trigger isL4 = new Trigger(() -> lastRequest == L4);

    public final Trigger isHoldingCoral = new Trigger(() -> manipulator.inputs.coralDistance.getValue() > Meters.of(0.2).getValue());

    // Command Factories

    public Command getSendToStateCommand(Supplier<SuperstructureState> superstructureState) {
        return Commands.sequence(
            getForceStateCommand(superstructureState),
            arm.getMoveToAngleCommand(() -> superstructureState.get().armAngle)
        );
    }

    public Command getForceStateCommand(Supplier<SuperstructureState> stateSupplier) {
        return Commands.runOnce(() -> {
            lastRequest = stateSupplier.get();
            Robot.drivebase.limits = lastRequest.driveLimits;
        });
    }

    public Trigger getStateAchievedTrigger(SuperstructureState target) {
        return new Trigger(() -> {
            Logger.recordOutput("target", target.name);
            boolean armAtSetpoint = arm.inputs.angle.epsilonEquals(
                target.armAngle,
                2.0.degrees
            );
            return armAtSetpoint;
        });
    }

    /// when shouldScore is true, coral will be released
    public Command getScoreCommand(BooleanSupplier shouldScore) {
        return Commands.sequence(
            getSendToStateCommand(() -> lastRequest),
            Commands.waitUntil(() -> getStateAchievedTrigger(lastRequest).getAsBoolean()),
            Commands.waitUntil(() -> !shouldScore.getAsBoolean()),
            Commands.waitUntil(() -> shouldScore.getAsBoolean()),
            manipulator.getSpinRollersOutCommand(),
            Commands.waitSeconds(1.0),
            manipulator.getStopRollersCommand()
            // getSendToStateCommand(() -> SuperstructureGoals.STOW),
            // Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.STOW).getAsBoolean()),
        );
    }

    public Command getProcessorCommand(BooleanSupplier shouldScore) {
        return Commands.sequence(
            Commands.waitUntil(() -> !shouldScore.getAsBoolean()),
            manipulator.getSpinRollersInCommand(),
            // Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> shouldScore.getAsBoolean()),
            manipulator.getStopRollersCommand()
        );
    }

    public Command getIntakeAlgaeCommand(BooleanSupplier done) {
        return Commands.sequence(
            getSendToStateCommand(() -> SuperstructureGoals.ALGAE_INTAKE),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.ALGAE_INTAKE).getAsBoolean()),
            manipulator.getSpinRollersOutCommand(),
            Commands.waitUntil(() -> !done.getAsBoolean()),
            Commands.waitUntil(() -> done.getAsBoolean()),
            manipulator.getStopRollersCommand()
        );
    }

    // new intake command
    public Command getIntakeCommand(BooleanSupplier shouldRetract) {
        return Commands.sequence(
            getSendToStateCommand(() -> SuperstructureGoals.GROUND),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.GROUND).getAsBoolean()),
            Commands.waitUntil(() -> !shouldRetract.getAsBoolean()),
            manipulator.getSpinRollersInCommand(),
            Commands.waitUntil(shouldRetract),
            manipulator.getStopRollersCommand(),
            getSendToStateCommand(() -> SuperstructureGoals.STOW),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.STOW).getAsBoolean())
        );
    }


    // score command
    public Command getScoreCommand2(BooleanSupplier readyToScore) {
        return Commands.sequence(
            getSendToStateCommand(() -> SuperstructureGoals.L1),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.L1).getAsBoolean()),
            // arm is at L1 scoring position
            Commands.waitUntil(() -> !readyToScore.getAsBoolean()),
            Commands.waitUntil(readyToScore),
            manipulator.getSpinRollersOutCommand(),
            Commands.waitSeconds(0.5),
            manipulator.getStopRollersCommand(),
            getSendToStateCommand(() -> SuperstructureGoals.STOW),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.STOW).getAsBoolean()),
        );
    }




    public Command getDealgaefyCommand(BooleanSupplier readyToDealgaefy) {
        return Commands.sequence(
            Commands.waitUntil(() -> !readyToDealgaefy.getAsBoolean()),
            getSendToStateCommand(() -> SuperstructureGoals.PRE_ALGAE_L2).onlyIf(() -> lastRequest == SuperstructureGoals.ALGAE_L2),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.PRE_ALGAE_L2).getAsBoolean()).onlyIf(() -> lastRequest == SuperstructureGoals.ALGAE_L2),
            getSendToStateCommand(() -> SuperstructureGoals.PRE_ALGAE_L3).onlyIf(() -> lastRequest == SuperstructureGoals.ALGAE_L3),
            Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.PRE_ALGAE_L3).getAsBoolean()).onlyIf(() -> lastRequest == SuperstructureGoals.ALGAE_L3),
            manipulator.getSpinRollersOutCommand().onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L2),
            manipulator.getSpinRollersInCommand().onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L3),
            Commands.waitUntil(() -> readyToDealgaefy.getAsBoolean()),
            manipulator.getStopRollersCommand()
            // getSendToStateCommand(() -> SuperstructureGoals.ALGAE_L2).onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L2),
            // Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.ALGAE_L2).getAsBoolean()).onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L2),
            // getSendToStateCommand(() -> SuperstructureGoals.ALGAE_L3).onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L3),
            // Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.ALGAE_L3).getAsBoolean()).onlyIf(() -> lastRequest == SuperstructureGoals.PRE_ALGAE_L3),
        );
    }

    // todo, will need to release while moving up to throw algae
    // public Command getReleaseAlgaeCommand() {
    //     return Commands.sequence(
    //         getSendToStateCommand(() -> lastRequest),
    //         Commands.waitUntil(() -> getStateAchievedTrigger(lastRequest).getAsBoolean()),
    //         manipulator.getSpinRollersInCommand(),
    //         Commands.waitSeconds(10.0),
    //         manipulator.getStopRollersCommand(),
    //         getSendToStateCommand(() -> SuperstructureGoals.STOW),
    //         Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.STOW).getAsBoolean()),
    //     );
    // }

    public void periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureState.struct, lastRequest);
        Logger.recordOutput("superstructure/setpoint/name", lastRequest.name);
    }
}
