package frc.robot.subsystems.superstructure.manipulator

import edu.wpi.first.units.measure.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ManipulatorIO {
    class ManipulatorInputs: LoggableInputs {
        var isLeftRollerConnected: Boolean = false
        var isRightRollerConnected: Boolean = false

        val leftRollerAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val leftRollerAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val leftRollerStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val leftRollerTorqueCurrent: MutCurrent = Amps.zero().mutableCopy()

        val rightRollerAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val rightRollerAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val rightRollerStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val rightRollerTorqueCurrent: MutCurrent = Amps.zero().mutableCopy()

        var coralDistance: MutDistance = Meters.zero().mutableCopy()
        
        override fun toLog(table: LogTable) {
            table.put("isLeftRollerConnected", isLeftRollerConnected)
            table.put("isRightRollerConnected", isRightRollerConnected)
            table.put("leftRollerAngularVelocity", leftRollerAngularVelocity)
            table.put("leftRollerAppliedVoltage", leftRollerAppliedVoltage)
            table.put("leftRollerStatorCurrent", leftRollerStatorCurrent)
            table.put("leftRollerTorqueCurrent", leftRollerTorqueCurrent)
            table.put("rightRollerAngularVelocity", rightRollerAngularVelocity)
            table.put("rightRollerAppliedVoltage", rightRollerAppliedVoltage)
            table.put("rightRollerStatorCurrent", rightRollerStatorCurrent)
            table.put("rightRollerTorqueCurrent", rightRollerTorqueCurrent)
            table.put("coralDistance", coralDistance)
        }
        
        override fun fromLog(table: LogTable) {
            isLeftRollerConnected = table.get("isLeftRollerConnected", isLeftRollerConnected)
            isRightRollerConnected = table.get("isRightRollerConnected", isRightRollerConnected)
            leftRollerAngularVelocity.mut_replace(table.get("leftRollerAngularVelocity", leftRollerAngularVelocity))
            leftRollerAppliedVoltage.mut_replace(table.get("leftRollerAppliedVoltage", leftRollerAppliedVoltage))
            leftRollerStatorCurrent.mut_replace(table.get("leftRollerStatorCurrent", leftRollerStatorCurrent))
            leftRollerTorqueCurrent.mut_replace(table.get("leftRollerTorqueCurrent", leftRollerTorqueCurrent))
            rightRollerAngularVelocity.mut_replace(table.get("rightRollerAngularVelocity", rightRollerAngularVelocity))
            rightRollerAppliedVoltage.mut_replace(table.get("rightRollerAppliedVoltage", rightRollerAppliedVoltage))
            rightRollerStatorCurrent.mut_replace(table.get("rightRollerStatorCurrent", rightRollerStatorCurrent))
            rightRollerTorqueCurrent.mut_replace(table.get("rightRollerTorqueCurrent", rightRollerTorqueCurrent))
            coralDistance.mut_replace(table.get("coralDistance", coralDistance))
        }
    }
    
    fun updateInputs(inputs: ManipulatorInputs) {}
    fun setRollerVoltage(voltage: Voltage) {}
    fun setLeftRightRollerVoltages(leftVoltage: Voltage, rightVoltage: Voltage) {}
    fun setRollerTorqueCurrent(current: Current) {}
    fun setLeftRightRollerTorqueCurrents(leftCurrent: Current, rightCurrent: Current) {}

    fun stopRoller() {}
}