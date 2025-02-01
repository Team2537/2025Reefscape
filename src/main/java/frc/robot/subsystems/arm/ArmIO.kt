package frc.robot.subsystems.arm

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ArmIO {
    class ArmInputs : LoggableInputs {
        /** The position of the motor(s), measured relatively by the motor(s)'s encoder. */
        val motorRelativePosition: MutAngle = Rotations.zero().mutableCopy()

        /** The position of the motor(s), measured absolutely. */
        val motorAbsolutePosition: MutAngle = Rotations.zero().mutableCopy()

        /** The velocity of the motor(s)'s rotation. */
        val motorVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()

        /** The voltage supplied to the motor(s). */
        val motorSupplyVoltage: MutVoltage = Volts.zero().mutableCopy()

        /** The voltage the motor(s) is/are using. */
        val motorVoltage: MutVoltage = Volts.zero().mutableCopy()

        /** The current applied to the motor(s). */
        val motorStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("motorRelativePosition", motorRelativePosition)
            table.put("motorAbsolutePosition", motorAbsolutePosition)
            table.put("motorVelocity", motorVelocity)
            table.put("motorSupplyVoltage", motorSupplyVoltage)
            table.put("motorVoltage", motorVoltage)
            table.put("motorStatorCurrent", motorStatorCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            motorRelativePosition.mut_replace(table.get("motorRelativePosition", motorRelativePosition))
            motorAbsolutePosition.mut_replace(table.get("motorAbsolutePosition", motorAbsolutePosition))
            motorVelocity.mut_replace(table.get("motorVelocity", motorVelocity))
            motorSupplyVoltage.mut_replace(table.get("motorSupplyVoltage", motorSupplyVoltage))
            motorVoltage.mut_replace(table.get("motorVoltage", motorVoltage))
            motorStatorCurrent.mut_replace(table.get("motorStatorCurrent", motorStatorCurrent))
        }
    }

    /**
     * Updates the given inputs with data from the arm systems.
     *
     * @param inputs The inputs to update.
     */
    fun updateInputs(inputs: ArmInputs) {}

    /**
     * Gives the arm a setpoint angle. The given angle is an angle of
     * elevation (relative to the horizontal)
     *
     * @param angle The desired angle of elevation.
     */
    fun setAngle(angle: Angle) {}

    /**
     * Sets the motor to run at a specific voltage.
     *
     * @param voltage The desired voltage.
     */
    fun setVoltage(voltage: Voltage) {}

    /**
     * Stops the motor.
     */
    fun stop() {}
}