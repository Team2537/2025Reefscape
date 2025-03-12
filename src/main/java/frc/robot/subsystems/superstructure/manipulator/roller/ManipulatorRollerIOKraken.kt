package frc.robot.subsystems.superstructure.manipulator.roller

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.reduxrobotics.sensors.canandcolor.Canandcolor
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.Voltage
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains

class ManipulatorRollerIOKraken(
    motorID: Int,
    sensorID: Int,
    pidGains: PIDGains,
    ffGains: FeedforwardGains,
    gearing: Double,
    flywheelRadius: Distance
): ManipulatorRollerIO {
    private val motor: TalonFX = TalonFX(motorID).apply {
        val config = TalonFXConfiguration()
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake
        config.Slot0.kP = pidGains.kP
        config.Slot0.kI = pidGains.kI
        config.Slot0.kD = pidGains.kD
        config.Slot0.kS = ffGains.kS
        config.Slot0.kV = ffGains.kV
        config.Slot0.kA = ffGains.kA
        
        config.Feedback.SensorToMechanismRatio = gearing
        
        configurator.apply(config)
    }
    
    private val sensor: Canandcolor = Canandcolor(sensorID)
    
    private val position = motor.position.clone()
    private val velocity = motor.velocity.clone()
    private val current = motor.statorCurrent.clone()
    private val voltage = motor.motorVoltage.clone()
    
    private val openLoopRequest: VoltageOut = VoltageOut(0.0)
    private val closedLoopVelocityRequest: VelocityVoltage = VelocityVoltage(0.0)
    
    override fun updateInputs(inputs: ManipulatorRollerIO.ManipulatorRollerInputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            position, velocity, current, voltage
        ).isOK
        
        inputs.angularVelocity.mut_replace(velocity.value)
        inputs.angularPosition.mut_replace(position.value)
        inputs.statorCurrent.mut_replace(current.value)
        inputs.appliedVoltage.mut_replace(voltage.value)
        inputs.coralDistance.mut_replace(sensor.proximity, Inches)
    }
    
    override fun setVoltage(voltage: Voltage) {
        motor.setControl(openLoopRequest.withOutput(voltage))
    }
    
    override fun setTargetVelocity(velocity: AngularVelocity) {
        motor.setControl(closedLoopVelocityRequest.withVelocity(velocity))
    }
}