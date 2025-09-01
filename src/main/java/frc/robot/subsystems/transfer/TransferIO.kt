package frc.robot.subsystems.transfer

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface TransferIO {
    class TransferInputs: LoggableInputs {
        val velocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val appliedVoltage: MutVoltage = Volt.zero().mutableCopy()
        val statorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val coralDistance: MutDistance = Meters.zero().mutableCopy()
        
        // TODO: add digital IR sensor for coral detection

        override fun fromLog(table: LogTable) {
            velocity.mut_replace(table.get("velocity", velocity))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
            coralDistance.mut_replace(table.get("coralDistance", coralDistance))
        }

        override fun toLog(table: LogTable) {
            table.put("velocity", velocity)
            table.put("appliedVoltage", appliedVoltage)
            table.put("statorCurrent", statorCurrent)
            table.put("coralDistance", coralDistance)
        }
    }

    fun updateInputs(inputs: TransferInputs) {}
    
    fun setVoltage(voltage: Voltage) {}

    fun setBrakeMode(brake: Boolean) {}
}