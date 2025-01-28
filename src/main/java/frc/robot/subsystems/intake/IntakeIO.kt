package frc.robot.subsystems.intake

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.reduxrobotics.sensors.canandcolor.ColorData

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        /** Can be used to determine how far target is from plate */
        var distanceFromPlateSensor: MutDistance = Meters.zero().mutableCopy()
        
        /** Can be used to detect if plate contains an algae */
        var doesDistanceMatchAlgaePlateSensor: Boolean = false
        
        /** Can be used to detect if plate contains a coral */
        var doesDistanceMatchCoralPlateSensor: Boolean = false
        
        /** Can be used to store color of algae and coral, making it easier for robot to differentiate between the two */
        var colorFromPlateSensor: ColorData = ColorData(0.0, 0.0, 0.0)
        
        /** Color can be used to determine if plate contains an algae */
        var doesColorMatchAlgaePlateSensor: Boolean = false 
        
        /** Color can be used to determine if plate contains a coral */
        var doesColorMatchCoralPlateSensor: Boolean = false
        
        /** Can be used to understand angular velocity of intake pivot arm */
        var pivotAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var pivotSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        var pivotMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        
        /** May be used to determine when to stop pivoting */
        var pivotStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        
        /** Can be used to understand angular velocity of left wheel */
        var leftWheelAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var leftSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        var leftMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        
        /** Can be used to determine if game piece is against plate */
        var leftStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        
        /** Can be used to understand angular velocity of right wheel */
        var rightWheelAngularVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var rightSupplyVoltage: MutVoltage = Volt.zero().mutableCopy()
        var rightMotorVoltage: MutVoltage = Volt.zero().mutableCopy()
        
        /** Can be used to determine if game piece is against plate */
        var rightStatorCurrent: MutCurrent = Amps.zero().mutableCopy()

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("distanceFromPlateSensor", distanceFromPlateSensor)
            table.put("doesDistanceMatchAlgaePlateSensor", doesDistanceMatchAlgaePlateSensor)
            table.put("doesDistanceMatchCoralPlateSensor", doesDistanceMatchCoralPlateSensor)
            table.put("colorFromPlateSensor", ColorData.struct, colorFromPlateSensor)
            table.put("doesColorMatchAlgaePlateSensor", doesColorMatchAlgaePlateSensor)
            table.put("doesColorMatchCoralPlateSensor", doesColorMatchCoralPlateSensor)
            
            table.put("pivotAngularVelocity", pivotAngularVelocity)
            table.put("pivotSupplyVoltage", pivotSupplyVoltage)
            table.put("pivotMotorVoltage", pivotMotorVoltage)
            table.put("pivotStatorCurrent", pivotStatorCurrent)
            
            table.put("leftWheelAngularVelocity", leftWheelAngularVelocity)
            table.put("leftSupplyVoltage", leftSupplyVoltage)
            table.put("leftMotorVoltage", leftMotorVoltage)
            table.put("leftStatorCurrent", leftStatorCurrent)
            
            table.put("rightWheelAngularVelocity", rightWheelAngularVelocity)
            table.put("rightSupplyVoltage", rightSupplyVoltage)
            table.put("rightMotorVoltage", rightMotorVoltage)
            table.put("rightStatorCurrent", rightStatorCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            doesDistanceMatchAlgaePlateSensor = table.get("doesDistanceMatchAlgaePlateSensor", doesDistanceMatchAlgaePlateSensor)
            doesDistanceMatchCoralPlateSensor = table.get("doesDistanceMatchCoralPlateSensor", doesDistanceMatchCoralPlateSensor)
            colorFromPlateSensor = table.get("colorFromPlateSensor", ColorData.struct, colorFromPlateSensor)
            doesColorMatchAlgaePlateSensor = table.get("doesColorMatchAlgaePlateSensor", doesColorMatchAlgaePlateSensor)
            doesColorMatchCoralPlateSensor = table.get("doesColorMatchCoralPlateSensor", doesColorMatchCoralPlateSensor)
            
            distanceFromPlateSensor.mut_replace(table.get("distanceFromPlateSensor", distanceFromPlateSensor))
            pivotAngularVelocity.mut_replace(table.get("pivotAngularVelocity", pivotAngularVelocity))
            pivotSupplyVoltage.mut_replace(table.get("pivotSupplyVoltage", pivotSupplyVoltage))
            pivotMotorVoltage.mut_replace(table.get("pivotMotorVoltage", pivotMotorVoltage))
            pivotStatorCurrent.mut_replace(table.get("pivotStatorCurrent", pivotStatorCurrent))
            
            leftWheelAngularVelocity.mut_replace(table.get("leftWheelAngularVelocity", leftWheelAngularVelocity))
            leftSupplyVoltage.mut_replace(table.get("leftSupplyVoltage", leftSupplyVoltage))
            leftMotorVoltage.mut_replace(table.get("leftMotorVoltage", leftMotorVoltage))
            leftStatorCurrent.mut_replace(table.get("leftStatorCurrent", leftStatorCurrent))
            
            rightWheelAngularVelocity.mut_replace(table.get("rightWheelAngularVelocity", rightWheelAngularVelocity))
            rightSupplyVoltage.mut_replace(table.get("rightSupplyVoltage", rightSupplyVoltage))
            rightMotorVoltage.mut_replace(table.get("rightMotorVoltage", rightMotorVoltage))
            rightStatorCurrent.mut_replace(table.get("rightStatorCurrent", rightStatorCurrent))
        }
    }
    fun updateInputs(inputs: IntakeInputs) {}
    
    // Output functions
    /** Sets the voltage for the left intake wheel */
    fun leftSetVoltage(voltage: Voltage)
    
    /** Stops the left intake wheel */
    fun leftStop()
    
    /** Sets the voltage for the right intake wheel */
    fun rightSetVoltage(voltage: Voltage)
    
    /** Stops the right intake wheel */
    fun rightStop()
    
    /** Sets the voltage for the pivot mechanism */
    fun pivotSetVoltage(voltage: Voltage)
    
    /** Stops the pivot mechanism */
    fun pivotStop()
    
    /** Stops all intake motors (wheels and pivot) */
    fun stopAll()
}