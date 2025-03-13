package frc.robot.subsystems.superstructure.manipulator.roller

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ManipulatorRollerIO {
    class ManipulatorRollerInputs: LoggableInputs {
        var isMotorConnected: Boolean = false
        
        val angularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val angularPosition: MutAngle = Radians.zero().mutableCopy()
        val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val statorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var coralDistance: Double = 0.0
        
        override fun toLog(table: LogTable) {
            table.put("isMotorConnected", isMotorConnected)
            table.put("angularVelocity", angularVelocity)
            table.put("angularPosition", angularPosition)
            table.put("appliedVoltage", appliedVoltage)
            table.put("statorCurrent", statorCurrent)
            table.put("coralDistance", coralDistance)
        }
        
        override fun fromLog(table: LogTable) {
            isMotorConnected = table.get("isMotorConnected", isMotorConnected)
            angularVelocity.mut_replace(table.get("angularVelocity", angularVelocity))
            angularPosition.mut_replace(table.get("angularPosition", angularPosition))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
            coralDistance = table.get("coralDistance", coralDistance)
        }
    }
    
    fun updateInputs(inputs: ManipulatorRollerInputs) {}
    fun setVoltage(voltage: Voltage) {}
    fun setTargetVelocity(velocity: AngularVelocity) {}
}