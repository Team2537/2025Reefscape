package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import jdk.jfr.Enabled
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutDistance

interface GripperIO{
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
    fun applyVoltage(volts: Voltage)
    fun sendPos(angle: Angle)
    fun setBrake(brakeEnabled: Boolean)
    fun updateInputs(inputs: GripperInputs)
}