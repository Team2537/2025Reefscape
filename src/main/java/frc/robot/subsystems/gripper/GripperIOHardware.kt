package frc.robot.subsystems.gripper

import edu.wpi.first.units.measure.*
import edu.wpi.first.units.Units.*
import com.reduxrobotics.sensors.canandcolor.ColorData
import com.reduxrobotics.sensors.canandcolor.Canandcolor

class GripperIOHardware(
    private val sensorID: Int,
    private val motorID: Int,
    private val isMotorInverted: Boolean,
    // used to convert from proximity reading to distance. In inches. see: https://arc.net/l/quote/sokscgag
    private val sensorProximityMaximum: Double
) : GripperIO {
    // Hardware Components
    // 1. Motor controller for the gripper mechanism
    // private val gripperMotor: Motor // Type depends on the specific motor controller being used
    
    // 2. Distance sensor for detecting game pieces
    private val sensor: Canandcolor = Canandcolor(sensorID)
    
    override fun updateInputs(inputs: GripperIO.GripperInputs) {
        // convert from proximity reading 0 to 1 to inches
        inputs.distanceFromSensor = Inches.of(sensorProximityMaximum * sensor.getProximity()).mutableCopy()
        inputs.colorFromSensor = sensor.getColor();
        
        // Update motor state
        // - Position
        // - Velocity
        // - Applied voltage
        // inputs.gripperPosition = ...
        // inputs.gripperVelocity = ...
        // inputs.gripperAppliedVoltage = ...
    }
    
    override fun setVoltage(voltage: Voltage) {
        // Apply the specified voltage to the gripper motor
        // Include safety limits if needed
    }
    
    override fun setBrakeMode(enabled: Boolean) {
        // Set the brake mode of the motor controller
    }
}
