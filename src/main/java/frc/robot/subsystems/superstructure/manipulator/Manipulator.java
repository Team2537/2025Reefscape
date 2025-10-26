package frc.robot.subsystems.superstructure.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotType;
import org.littletonrobotics.junction.Logger;

/** Manipulator subsystem managing intake rollers. */
public final class Manipulator extends SubsystemBase {
  private final ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  public Manipulator() {
    this.io = switch (RobotType.MODE) {
      case REAL -> new ManipulatorIOKraken();
      case SIMULATION -> new ManipulatorIOSim();
      case REPLAY -> new ManipulatorIO() {
      };
    };
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  public Command getSpinRollersInCommand() {
    return Commands.runOnce(
        () -> io.setRollerTorqueCurrent(ManipulatorConstants.ROLLER_IN_TORQUE_CURRENT));
  }

  public Command getSpinRollersInSlowCommand() {
    return Commands.runOnce(
        () -> io.setRollerTorqueCurrent(
            ManipulatorConstants.ROLLER_IN_TORQUE_CURRENT.times(0.25)));
  }

  public Command getSpinRollersOutCommand() {
    return Commands.runOnce(
        () -> io.setRollerVoltage(ManipulatorConstants.ROLLER_OUT_VOLTAGE));
  }

  public Command getStopRollersCommand() {
    return Commands.runOnce(io::stopRoller);
  }

  public Trigger isDetectingGamePiece() {
    return new Trigger(
        () -> inputs.coralDistance.lt(ManipulatorConstants.DETECTION_DISTANCE_THRESHOLD));
  }

  public ManipulatorIO.ManipulatorIOInputs getInputs() {
    return inputs;
  }
}
