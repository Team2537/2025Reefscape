package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.swerve.Drivebase;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Coordinates arm and manipulator mechanisms. */
public final class Superstructure extends SubsystemBase {
  private final Drivebase drivebase;
  private final Arm arm;
  private final Manipulator manipulator;

  private SuperstructureState lastRequest = SuperstructureGoals.STOW;

  private final Trigger isL1 = new Trigger(() -> lastRequest == SuperstructureGoals.L1);
  private final Trigger readyToScore = new Trigger(() -> SmartDashboard.getBoolean("shouldScore", false));

  public Superstructure(Drivebase drivebase) {
    this.drivebase = drivebase;
    this.arm = new Arm();
    this.manipulator = new Manipulator();
  }

  public Command getArmSysIdCommand() {
    return Commands.sequence(
        arm.getDynamicSysID(Direction.kForward),
        arm.getDynamicSysID(Direction.kReverse),
        arm.getQuasistaticSysID(Direction.kForward),
        arm.getQuasistaticSysID(Direction.kReverse));
  }

  public Trigger isL1() {
    return isL1;
  }

  public Trigger isHoldingCoral() {
    double threshold = ManipulatorConstants.DETECTION_DISTANCE_THRESHOLD_METERS;
    return new Trigger(() -> manipulator.getInputs().coralDistanceMeters > threshold);
  }

  public Command getSendToStateCommand(Supplier<SuperstructureState> stateSupplier) {
    return Commands.sequence(
        getForceStateCommand(stateSupplier),
        arm.getMoveToAngleCommand(() -> stateSupplier.get().getArmAngle()));
  }

  public Command getForceStateCommand(Supplier<SuperstructureState> stateSupplier) {
    return Commands.runOnce(
        () -> {
          lastRequest = stateSupplier.get();
          drivebase.setLimits(lastRequest.getDriveLimits());
        });
  }

  public Trigger getStateAchievedTrigger(SuperstructureState target) {
    return new Trigger(
        () ->
            Math.abs(
                    arm.getAngle().minus(target.getArmAngle()).getDegrees())
                < 2.0);
  }

  public Command getScoreCommand(BooleanSupplier shouldScore) {
    return Commands.sequence(
        getSendToStateCommand(() -> lastRequest),
        Commands.waitUntil(() -> getStateAchievedTrigger(lastRequest).getAsBoolean()),
        Commands.waitUntil(() -> !shouldScore.getAsBoolean()),
        Commands.waitUntil(shouldScore::getAsBoolean),
        manipulator.getSpinRollersOutCommand(),
        Commands.waitSeconds(1.0),
        manipulator.getStopRollersCommand());
  }

  public Command getProcessorCommand(BooleanSupplier shouldScore) {
    return Commands.sequence(
        Commands.waitUntil(() -> !shouldScore.getAsBoolean()),
        manipulator.getSpinRollersInCommand(),
        Commands.waitUntil(shouldScore::getAsBoolean),
        manipulator.getStopRollersCommand());
  }

  public Command getIntakeAlgaeCommand(BooleanSupplier done) {
    return Commands.sequence(
        getSendToStateCommand(() -> SuperstructureGoals.ALGAE_INTAKE),
        Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.ALGAE_INTAKE).getAsBoolean()),
        manipulator.getSpinRollersOutCommand(),
        Commands.waitUntil(() -> !done.getAsBoolean()),
        Commands.waitUntil(done::getAsBoolean),
        manipulator.getStopRollersCommand());
  }

  public Command getIntakeCommand(BooleanSupplier shouldRetract) {
    return Commands.sequence(
        getSendToStateCommand(() -> SuperstructureGoals.GROUND),
        Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.GROUND).getAsBoolean()),
        Commands.waitUntil(() -> !shouldRetract.getAsBoolean()),
        manipulator.getSpinRollersInCommand(),
        Commands.waitUntil(shouldRetract::getAsBoolean),
        manipulator.getStopRollersCommand(),
        getSendToStateCommand(() -> SuperstructureGoals.STOW),
        Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.STOW).getAsBoolean()));
  }

  public Command getDealgaefyL2Command(BooleanSupplier readyToDealgaefy) {
    return Commands.sequence(
        Commands.waitUntil(() -> !readyToDealgaefy.getAsBoolean()),
        getSendToStateCommand(() -> SuperstructureGoals.PRE_ALGAE_L2),
        Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.PRE_ALGAE_L2).getAsBoolean()),
        Commands.deadline(
            Commands.waitUntil(readyToDealgaefy::getAsBoolean),
            manipulator.getSpinRollersOutCommand()),
        manipulator.getStopRollersCommand());
  }

  public Command getDealgaefyL3Command(BooleanSupplier readyToDealgaefy) {
    return Commands.sequence(
        Commands.waitUntil(() -> !readyToDealgaefy.getAsBoolean()),
        getSendToStateCommand(() -> SuperstructureGoals.PRE_ALGAE_L3),
        Commands.waitUntil(() -> getStateAchievedTrigger(SuperstructureGoals.PRE_ALGAE_L3).getAsBoolean()),
        Commands.deadline(
            Commands.waitUntil(readyToDealgaefy::getAsBoolean),
            manipulator.getSpinRollersInCommand()),
        manipulator.getStopRollersCommand());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("superstructure/setpoint", SuperstructureState.STRUCT, lastRequest);
    Logger.recordOutput("superstructure/setpoint/name", lastRequest.getName());
  }

  public Drivebase getDrivebase() {
    return drivebase;
  }

  public Arm getArm() {
    return arm;
  }

  public Manipulator getManipulator() {
    return manipulator;
  }

  public SuperstructureState getLastRequest() {
    return lastRequest;
  }
}
