package frc.robot.subsystems

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
        val angleFromAbsoluteEncoder: MutAngle = Rotations.zero().mutableCopy()
        val climbArmAngularVelocity: MutAngularVelocity = RotationsPerSecond.zero().mutableCopy()
        val climbArmSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        val climbArmMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        val climbArmStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val climbSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("angleFromAbsoluteEncoder", angleFromAbsoluteEncoder)
            table.put("climbArmAngularVelocity", climbArmAngularVelocity)
            table.put("climbArmSupplyVoltage", climbArmSupplyVoltage)
            table.put("climbArmMotorVoltage", climbArmMotorVoltage)
            table.put("climbArmStatorCurrent", climbArmStatorCurrent)
            table.put("climbSupplyCurrent", climbSupplyCurrent)
        }

        override fun fromLog(table: LogTable) {
            angleFromAbsoluteEncoder.mut_replace(table.get("angleFromAbsoluteEncoder", angleFromAbsoluteEncoder))
            climbArmAngularVelocity.mut_replace(table.get("climbArmAngularVelocity", climbArmAngularVelocity))
            climbArmSupplyVoltage.mut_replace(table.get("climbArmSupplyVoltage", climbArmSupplyVoltage))
            climbArmMotorVoltage.mut_replace(table.get("climbArmMotorVoltage", climbArmMotorVoltage))
            climbArmStatorCurrent.mut_replace(table.get("climbArmStatorCurrent", climbArmStatorCurrent))
            climbSupplyCurrent.mut_replace(table.get("climbSupplyCurrent", climbSupplyCurrent))
        }
    }
    fun climbArmSetVoltage(voltage: Voltage)
    fun climbArmStop()
    fun stopAll()
    fun setPos(angle: Angle)
    fun setBrake(brakeEnabled: Boolean)
}