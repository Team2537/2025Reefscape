package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.reduxrobotics.sensors.canandcolor.ColorData


// elevator has two motors, both on the same shaft. They are krakens and both have relative encoders. 
// We don't have any other sensors, so we will try to turn on the robot when the elevator carriage is at the bottom of the elevator.
// We may also use current homing.

interface ElevatorIO {
    class ElevatorInputs : LoggableInputs {

        var leftMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        var leftSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        var leftAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var leftStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var leftEncoderPosition: MutAngle = Radians.zero().mutableCopy()

        var rightMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        var rightSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        var rightAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var rightStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var rightEncoderPosition: MutAngle = Radians.zero().mutableCopy()

        var carriageHeight: MutDistance = Meters.zero().mutableCopy()
        var carriageVelocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("leftMotorVoltage", leftMotorVoltage)
            table.put("leftSupplyVoltage", leftSupplyVoltage)
            table.put("leftAngularVelocity", leftAngularVelocity)
            table.put("leftStatorCurrent", leftStatorCurrent)
            table.put("leftEncoderPosition", leftEncoderPosition)
            
            table.put("rightMotorVoltage", rightMotorVoltage)
            table.put("rightSupplyVoltage", rightSupplyVoltage)
            table.put("rightAngularVelocity", rightAngularVelocity)
            table.put("rightStatorCurrent", rightStatorCurrent)
            table.put("rightEncoderPosition", rightEncoderPosition)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            leftMotorVoltage.mut_replace(table.get("leftMotorVoltage", leftMotorVoltage))
            leftSupplyVoltage.mut_replace(table.get("leftSupplyVoltage", leftSupplyVoltage))
            leftAngularVelocity.mut_replace(table.get("leftAngularVelocity", leftAngularVelocity))
            leftStatorCurrent.mut_replace(table.get("leftStatorCurrent", leftStatorCurrent))
            leftEncoderPosition.mut_replace(table.get("leftEncoderPosition", leftEncoderPosition))

            rightMotorVoltage.mut_replace(table.get("rightMotorVoltage", rightMotorVoltage))
            rightSupplyVoltage.mut_replace(table.get("rightSupplyVoltage", rightSupplyVoltage))
            rightAngularVelocity.mut_replace(table.get("rightAngularVelocity", rightAngularVelocity))
            rightStatorCurrent.mut_replace(table.get("rightStatorCurrent", rightStatorCurrent))
            rightEncoderPosition.mut_replace(table.get("rightEncoderPosition", rightEncoderPosition))
        }
    }
    fun updateInputs(inputs: ElevatorInputs) {}
    
    // Output functions
    fun setElevatorVoltage(volts: Voltage) {}
    fun setElevatorHeightTarget(height: Distance) {}
    fun stop() {}
}
