package frc.robot.subsystems

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ArmIO {
    class ArmInputs: LoggableInputs {
        val armAngularVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()
        val armSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        val armMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        val armStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val armSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("armAngularVelocity", armAngularVelocity)
            table.put("armSupplyVoltage", armSupplyVoltage)
            table.put("armMotorVoltage", armMotorVoltage)
            table.put("armStatorCurrent", armStatorCurrent)
            table.put("armSupplyCurrent", armSupplyCurrent)
        }

        override fun fromLog(table: LogTable) {
            armAngularVelocity.mut_replace(table.get("armAngularVelocity", armAngularVelocity))
            armSupplyVoltage.mut_replace(table.get("armSupplyVoltage", armSupplyVoltage))
            armMotorVoltage.mut_replace(table.get("armMotorVoltage", armMotorVoltage))
            armStatorCurrent.mut_replace(table.get("armStatorVoltage", armStatorCurrent))
            armSupplyCurrent.mut_replace(table.get("armSupplyCurrent", armSupplyCurrent))
        }
    }
    fun armSetVoltage(voltage: Voltage)
    fun armStop()
    fun stopAll()
    fun senPos(angle: Angle)
    fun setBrake(brakeEnabled: Boolean)
}