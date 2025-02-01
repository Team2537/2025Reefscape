package frc.robot.subsystems.util

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Inputs for an arbitrary motor or group of linked motors.
 */
class MotorInputs : LoggableInputs {
    /** The position of the motor(s), measured relatively by the motor(s)'s encoder. */
    val relativePosition: MutAngle = Rotations.zero().mutableCopy()

    /** The position of the motor(s), measured absolutely. */
    val absolutePosition: MutAngle = Rotations.zero().mutableCopy()

    /** The velocity of the motor(s)'s rotation. */
    val velocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()

    /** The voltage supplied to the motor(s). */
    val supplyVoltage: MutVoltage = Volts.zero().mutableCopy()

    /** The voltage the motor(s) is/are using. */
    val outputVoltage: MutVoltage = Volts.zero().mutableCopy()

    /** The current applied to the motor(s). */
    val statorCurrent: MutCurrent = Amps.zero().mutableCopy()

    /**
     * Updates a LogTable with the data to log.
     */
    override fun toLog(table: LogTable) {
        table.put("relativePosition", relativePosition)
        table.put("absolutePosition", absolutePosition)
        table.put("velocity", velocity)
        table.put("supplyVoltage", supplyVoltage)
        table.put("outputVoltage", outputVoltage)
        table.put("statorCurrent", statorCurrent)
    }

    /**
     * Updates data based on a LogTable.
     */
    override fun fromLog(table: LogTable) {
        relativePosition.mut_replace(table.get("relativePosition", relativePosition))
        absolutePosition.mut_replace(table.get("absolutePosition", absolutePosition))
        velocity.mut_replace(table.get("velocity", velocity))
        supplyVoltage.mut_replace(table.get("supplyVoltage", supplyVoltage))
        outputVoltage.mut_replace(table.get("outputVoltage", outputVoltage))
        statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
    }
}