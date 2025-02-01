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
        // It's so clean though... (,_,)
//        /** The inputs for the motor that pivots the arm. */
//        val pivot: MotorInputs = MotorInputs()
//
//        /**
//         * Updates a LogTable with the data to log.
//         */
//        override fun toLog(table: LogTable) {
//            table.put("pivot", pivot)
//        }
//
//        /**
//         * Updates data based on a LogTable.
//         */
//        override fun fromLog(table: LogTable) {
//            table.get("pivot", pivot)
//        }
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
            table.put("motor/relativePosition", relativePosition)
            table.put("motor/absolutePosition", absolutePosition)
            table.put("motor/velocity", velocity)
            table.put("motor/supplyVoltage", supplyVoltage)
            table.put("motor/outputVoltage", outputVoltage)
            table.put("motor/statorCurrent", statorCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            relativePosition.mut_replace(table.get("motor/relativePosition", relativePosition))
            absolutePosition.mut_replace(table.get("motor/absolutePosition", absolutePosition))
            velocity.mut_replace(table.get("motor/velocity", velocity))
            supplyVoltage.mut_replace(table.get("motor/supplyVoltage", supplyVoltage))
            outputVoltage.mut_replace(table.get("motor/outputVoltage", outputVoltage))
            statorCurrent.mut_replace(table.get("motor/statorCurrent", statorCurrent))
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