package frc.robot.subsystems.gripper

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.units.Units.*

interface GripperIO {
    class GripperInputs: LoggableInputs {
        val gripperDistance: MutAngle = Rotations.zero().mutableCopy()
        val gripperPosition: MutAngle = Rotations.zero().mutableCopy()
        val gripperVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()
        val gripperAppliedVoltage: MutVoltage = Volt.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("gripperDistance", gripperDistance)
            table.put("gripperPosition", gripperPosition)
            table.put("gripperVelocity", gripperVelocity)
            table.put("gripperAppliedVoltage", gripperAppliedVoltage)
        }

        override fun fromLog(table: LogTable) {
            gripperDistance.mut_replace(table.get("gripperDistance", gripperDistance))
            gripperPosition.mut_replace(table.get("gripperPosition", gripperPosition))
            gripperVelocity.mut_replace(table.get("gripperVelocity", gripperVelocity))
            gripperAppliedVoltage.mut_replace(table.get("gripperAppliedVoltage", gripperAppliedVoltage))
        }
    }

    /**
     * Updates the input values for the gripper subsystem
     * @param inputs The inputs object to update with current gripper state
     */
    fun updateInputs(inputs: GripperInputs)

    /**
     * Sets the voltage to be applied to the gripper motor
     * @param voltage The voltage to apply
     */
    fun setVoltage(voltage: Voltage)

    /**
     * Enables or disables brake mode on the gripper motor
     * @param enabled True to enable brake mode, false for coast
     */
    fun setBrakeMode(enabled: Boolean)
}