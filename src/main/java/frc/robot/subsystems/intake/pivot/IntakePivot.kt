package frc.robot.subsystems.intake.pivot

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.IntakeConstants.PivotConstants.DOWN_ANGLE
import frc.robot.Constants.IntakeConstants.PivotConstants.FF_GAINS
import frc.robot.Constants.IntakeConstants.PivotConstants.GEARING
import frc.robot.Constants.IntakeConstants.PivotConstants.IS_MOTOR_INVERTED
import frc.robot.Constants.IntakeConstants.PivotConstants.KG
import frc.robot.Constants.IntakeConstants.PivotConstants.MOI
import frc.robot.Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_ACCELERATION
import frc.robot.Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_CRUISE_VELOCITY
import frc.robot.Constants.IntakeConstants.PivotConstants.MOTOR_ID
import frc.robot.Constants.IntakeConstants.PivotConstants.PID_GAINS
import frc.robot.Constants.IntakeConstants.PivotConstants.UP_ANGLE
import frc.robot.RobotType
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotInputs
import org.littletonrobotics.junction.Logger

class IntakePivot : SubsystemBase() {
    private val io: IntakePivotIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> IntakePivotIOKraken(
            MOTOR_ID,
            GEARING,
            IS_MOTOR_INVERTED,
            PID_GAINS,
            FF_GAINS,
            KG,
            MOTION_MAGIC_ACCELERATION,
            MOTION_MAGIC_CRUISE_VELOCITY
        )

        RobotType.Mode.SIMULATION -> IntakePivotIOSim(
            DCMotor.getKrakenX60Foc(1),
            GEARING,
            MOI,
            PID_GAINS,
            FF_GAINS,
            KG
        )

        RobotType.Mode.REPLAY -> object : IntakePivotIO {}
    }

    private val inputs: IntakePivotInputs = IntakePivotInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IntakePivot", inputs)

        // TODO: add logging for outputs
    }

    // move intake arm into the down position, from the constants file
    /**
     * Creates a command that sets the arm motor(s) setpoint to the
     * [down position][DOWN_ANGLE].
     *
     * @return A command that moves the arm down.
     *
     * @see DOWN_ANGLE
     * @see getSetPositionCommand
     */
    fun getSlamDownCommand(): Command {
        return getSetPositionCommand(DOWN_ANGLE)
    }

    // move intake arm into the up position, from the constants file
    /**
     * Creates a command that sets the arm motor(s) setpoint to the
     * [up position][UP_ANGLE].
     *
     * @return A command that moves the arm up.
     *
     * @see UP_ANGLE
     * @see getSetPositionCommand
     */
    fun getSlamUpCommand(): Command {
        return getSetPositionCommand(UP_ANGLE)
    }

    /**
     * Creates a new command that sets the motor(s)'s output voltage to
     * whatever is specified at that moment by the given supplier.
     *
     * @param voltage The supplier of what voltage the motor should be given.
     * This supplier can be expected to be invoked every iteration this command
     * is in use.
     * @return A command that sends a specific amount of voltage to the pivot
     * motor(s).
     */
    fun getSetVoltageCommand(voltage: () -> Voltage): Command {
        return Commands.run ({io.setVoltage(voltage())}, this)
    }

    /**
     * Creates a new command that sets the motor(s)'s output voltage to
     * the given voltage.
     *
     * Mutating the given measure will not affect the returned command.
     *
     * @param voltage The amount of voltage this command sends to the motor(s).
     * @return A command that sends a specific amount of voltage to the pivot
     * motor(s).
     */
    fun getSetVoltageCommand(voltage: Voltage): Command {
        return Commands.runOnce ({io.setVoltage(voltage.copy())}, this)
    }

    /**
     * Creates a new command that sets the motor(s)'s setpoint to whatever
     * arm position is specified at that moment by the given supplier.
     *
     * @param position The supplier of what angle the motor should move towards.
     * This supplier can be expected to be invoked every iteration this command
     * is in use.
     * @return A command that moves the arm to a specific angle.
     */
    fun getSetPositionCommand(position: () -> Angle): Command {
        return Commands.run ({io.setTargetAngle(position())}, this)
    }

    /**
     * Creates a new command that sets the motor(s)'s output voltage to
     * the given voltage.
     *
     * Mutating the given measure will not affect the returned command.
     *
     * @param position The position this command sends the arm to.
     * @return A command that sends a specific amount of voltage to the pivot
     * motor(s).
     */
    fun getSetPositionCommand(position: Angle): Command {
        return Commands.runOnce ({io.setTargetAngle(position.copy())}, this)
    }
}