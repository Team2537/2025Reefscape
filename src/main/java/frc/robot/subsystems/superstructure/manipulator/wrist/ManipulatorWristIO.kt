package frc.robot.subsystems.superstructure.manipulator.wrist

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ManipulatorWristIO {
    class ManipulatorWristInputs : LoggableInputs {
        var isConnected = false

        val angle: MutAngle = Radians.zero().mutableCopy()
        val velocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val statorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("isConnected", isConnected)
            table.put("angle", angle)
            table.put("velocity", velocity)
            table.put("statorCurrent", statorCurrent)
            table.put("appliedVoltage", appliedVoltage)
        }

        override fun fromLog(table: LogTable) {
            isConnected = table.get("isConnected", isConnected)
            angle.mut_replace(table.get("angle", angle))
            velocity.mut_replace(table.get("velocity", velocity))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
        }
    }

    fun updateInputs(inputs: ManipulatorWristInputs) {}
    fun setVoltage(voltage: Voltage) {}
    fun setTargetPosition(targetAngle: Angle) {}
}