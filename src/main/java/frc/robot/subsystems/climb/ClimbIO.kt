package frc.robot.subsystems.climb

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ClimbIO{
    class ClimbArmInputs: LoggableInputs {
        val absoluteAngle: MutAngle = Rotations.zero().mutableCopy()
        val angularVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()
        val supplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        val motorVoltage: MutVoltage = Volt.zero().mutableCopy()
        val statorCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("absoluteAngle", absoluteAngle)
            table.put("angularVelocity", angularVelocity)
            table.put("supplyVoltage", supplyVoltage)
            table.put("motorVoltage", motorVoltage)
            table.put("statorCurrent", statorCurrent)
        }

        override fun fromLog(table: LogTable) {
            absoluteAngle.mut_replace(table.get("absoluteAngle", absoluteAngle))
            angularVelocity.mut_replace(table.get("angularVelocity", angularVelocity))
            supplyVoltage.mut_replace(table.get("supplyVoltage", supplyVoltage))
            motorVoltage.mut_replace(table.get("motorVoltage", motorVoltage))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
        }
    }
    fun updateInputs(inputs: ClimbArmInputs) {}
    fun setVoltage(voltage: Voltage) {}
    fun setBrakeMode(brakeEnabled: Boolean) {}
    fun stop() {}
}