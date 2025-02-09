package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutVoltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface RollerIO {
    class RollerInputs: LoggableInputs {
        val position: MutAngle = Radians.zero().mutableCopy()
        val velocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        val appliedVoltage: MutVoltage = Volt.zero().mutableCopy()
        val statorCurrent: MutCurrent = Amps.zero().mutableCopy()

        override fun fromLog(table: LogTable) {
            position.mut_replace(table.get("position", position))
            velocity.mut_replace(table.get("velocity", velocity))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
        }

        override fun toLog(table: LogTable) {
            table.put("position", position)
            table.put("velocity", velocity)
            table.put("appliedVoltage", appliedVoltage)
            table.put("statorCurrent", statorCurrent)
        }
    }
}