package frc.robot.subsystems.superstructure.arm

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ArmIO {
    class ArmInputs : LoggableInputs {
        var isLeftMotorConnected: Boolean = false
        var isRightMotorConnected: Boolean = false

        val angle: MutAngle = Radians.zero().mutableCopy()
        val angularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val leftStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val rightStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("isLeftMotorConnected", isLeftMotorConnected)
            table.put("isRightMotorConnected", isRightMotorConnected)
            table.put("angle", angle)
            table.put("angularVelocity", angularVelocity)
            table.put("appliedVoltage", appliedVoltage)
            table.put("leftStatorCurrent", leftStatorCurrent)
            table.put("rightStatorCurrent", rightStatorCurrent)
        }

        override fun fromLog(table: LogTable) {
            isLeftMotorConnected = table.get("isLeftMotorConnected", isLeftMotorConnected)
            isRightMotorConnected = table.get("isRightMotorConnected", isRightMotorConnected)
            angle.mut_replace(table.get("angle", angle))
            angularVelocity.mut_replace(table.get("angularVelocity", angularVelocity))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
            leftStatorCurrent.mut_replace(table.get("leftStatorCurrent", leftStatorCurrent))
            rightStatorCurrent.mut_replace(table.get("rightStatorCurrent", rightStatorCurrent))
        }
    }

    fun updateInputs(inputs: ArmInputs) {}
    fun setVoltage(voltage: Voltage) {}
    fun setTargetAngle(angle: Angle) {}
    fun setBrakeMode(brake: Boolean) {}
    fun stop() {}
}


