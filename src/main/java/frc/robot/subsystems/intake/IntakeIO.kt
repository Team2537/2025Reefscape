package frc.robot.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.units.measure.MutDistance


interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        // CAN and color
        val coralDistance: MutDistance = Meters.zero().mutableCopy()

        // pivot
        val pivotLeftPosition: MutAngle = Radians.zero().mutableCopy()
        val pivotLeftVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val pivotLeftAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val pivotLeftStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val pivotRightPosition: MutAngle = Radians.zero().mutableCopy()
        val pivotRightVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val pivotRightAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val pivotRightStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        val pivotSetAngle: MutAngle = Radians.zero().mutableCopy()

        // rollers
        val rollerTargetVoltage: MutVoltage = Volts.zero().mutableCopy()
        val rollerVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val rollerAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
        val rollerStatorCurrent: MutCurrent = Amps.zero().mutableCopy()


        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("coralDistance", coralDistance)
            table.put("pivotLeftPosition", pivotLeftPosition)
            table.put("pivotLeftVelocity", pivotLeftVelocity)
            table.put("pivotLeftAppliedVoltage", pivotLeftAppliedVoltage)
            table.put("pivotLeftStatorCurrent", pivotLeftStatorCurrent)
            table.put("pivotRightPosition", pivotRightPosition)
            table.put("pivotRightVelocity", pivotRightVelocity)
            table.put("pivotRightAppliedVoltage", pivotRightAppliedVoltage)
            table.put("pivotRightStatorCurrent", pivotRightStatorCurrent)
            table.put("pivotSetAngle", pivotSetAngle)

            table.put("rollerVelocity", rollerVelocity)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage)
            table.put("rollerStatorCurrent", rollerStatorCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            coralDistance.mut_replace(table.get("coralDistance", coralDistance))
            pivotLeftPosition.mut_replace(table.get("pivotLeftPosition", pivotLeftPosition))
            pivotLeftVelocity.mut_replace(table.get("pivotLeftVelocity", pivotLeftVelocity))
            pivotLeftAppliedVoltage.mut_replace(table.get("pivotLeftAppliedVoltage", pivotLeftAppliedVoltage))
            pivotLeftStatorCurrent.mut_replace(table.get("pivotLeftStatorCurrent", pivotLeftStatorCurrent))
            pivotRightPosition.mut_replace(table.get("pivotRightPosition", pivotRightPosition))
            pivotRightVelocity.mut_replace(table.get("pivotRightVelocity", pivotRightVelocity))
            pivotRightAppliedVoltage.mut_replace(table.get("pivotRightAppliedVoltage", pivotRightAppliedVoltage))
            pivotRightStatorCurrent.mut_replace(table.get("pivotRightStatorCurrent", pivotRightStatorCurrent))
            pivotSetAngle.mut_replace(table.get("pivotSetAngle", pivotSetAngle))
            
            rollerVelocity.mut_replace(table.get("rollerVelocity", rollerVelocity))
            rollerAppliedVoltage.mut_replace(table.get("rollerAppliedVoltage", rollerAppliedVoltage))
            rollerStatorCurrent.mut_replace(table.get("rollerStatorCurrent", rollerStatorCurrent))
        }
    }
    fun updateInputs(inputs: IntakeInputs) {}
    
    // output functions
    fun setPivotAngle(angle: Angle) {}
    fun setPivotVoltage(voltage: Voltage) {}
    fun setRollerVoltage(voltage: Voltage) {}

    // set brake mode
    fun setPivotBrakeMode(brake: Boolean) {}

    // stop all
    fun stopRoller() {}

}
