package frc.robot.subsystems.superstructure.manipulator

import edu.wpi.first.units.measure.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ManipulatorIO {
    class ManipulatorInputs: LoggableInputs {
        var isRollerMotorConnected: Boolean = false
        var isPivotMotorConnected: Boolean = false
        
        val rollerAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val rollerAngularPosition: MutAngle = Radians.zero().mutableCopy()
        val rollerAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val rollerStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        val pivotAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val pivotAngularPosition: MutAngle = Radians.zero().mutableCopy()
        val pivotAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val pivotStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        var coralDistance: MutDistance = Meters.zero().mutableCopy()
        
        override fun toLog(table: LogTable) {
            table.put("isRollerMotorConnected", isRollerMotorConnected)
            table.put("isPivotMotorConnected", isPivotMotorConnected)
            table.put("rollerAngularVelocity", rollerAngularVelocity)
            table.put("rollerAngularPosition", rollerAngularPosition)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage)
            table.put("rollerStatorCurrent", rollerStatorCurrent)
            table.put("pivotAngularVelocity", pivotAngularVelocity)
            table.put("pivotAngularPosition", pivotAngularPosition)
            table.put("pivotAppliedVoltage", pivotAppliedVoltage)
            table.put("pivotStatorCurrent", pivotStatorCurrent)
            table.put("coralDistance", coralDistance)
        }
        
        override fun fromLog(table: LogTable) {
            isRollerMotorConnected = table.get("isRollerMotorConnected", isRollerMotorConnected)
            isPivotMotorConnected = table.get("isPivotMotorConnected", isPivotMotorConnected)
            rollerAngularVelocity.mut_replace(table.get("rollerAngularVelocity", rollerAngularVelocity))
            rollerAngularPosition.mut_replace(table.get("rollerAngularPosition", rollerAngularPosition))
            rollerAppliedVoltage.mut_replace(table.get("rollerAppliedVoltage", rollerAppliedVoltage))
            rollerStatorCurrent.mut_replace(table.get("rollerStatorCurrent", rollerStatorCurrent))
            pivotAngularVelocity.mut_replace(table.get("pivotAngularVelocity", pivotAngularVelocity))
            pivotAngularPosition.mut_replace(table.get("pivotAngularPosition", pivotAngularPosition))
            pivotAppliedVoltage.mut_replace(table.get("pivotAppliedVoltage", pivotAppliedVoltage))
            pivotStatorCurrent.mut_replace(table.get("pivotStatorCurrent", pivotStatorCurrent))
            coralDistance.mut_replace(table.get("coralDistance", coralDistance))
        }
    }
    
    fun updateInputs(inputs: ManipulatorInputs) {}
    fun setRollerVoltage(voltage: Voltage) {}
    fun setPivotVoltage(voltage: Voltage) {}
    fun setPivotTargetAngle(angle: Angle) {}

    fun stopRoller() {}

    fun setPivotBrakeMode(brake: Boolean) {}
}