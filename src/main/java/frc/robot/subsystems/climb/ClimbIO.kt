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
        val climbArmAngularVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()
        val climbArmSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        val climbArmMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        val climbArmStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("climbArmAbsoluteAngle", absoluteAngle)
            table.put("climbArmAngularVelocity", climbArmAngularVelocity)
            table.put("climbArmSupplyVoltage", climbArmSupplyVoltage)
            table.put("climbArmMotorVoltage", climbArmMotorVoltage)
            table.put("climbArmStatorCurrent", climbArmStatorCurrent)
        }

        override fun fromLog(table: LogTable) {
            absoluteAngle.mut_replace(table.get("climbArmAbsoluteAngle", absoluteAngle))
            climbArmAngularVelocity.mut_replace(table.get("climbArmAngularVelocity", climbArmAngularVelocity))
            climbArmSupplyVoltage.mut_replace(table.get("climbArmSupplyVoltage", climbArmSupplyVoltage))
            climbArmMotorVoltage.mut_replace(table.get("climbArmMotorVoltage", climbArmMotorVoltage))
            climbArmStatorCurrent.mut_replace(table.get("climbArmStatorCurrent", climbArmStatorCurrent))
        }
    }
    fun setVoltage(voltage: Voltage)
    fun setBrakeMode(brakeEnabled: Boolean)
    fun stop()
}