package frc.robot.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage

class ElevatorIOHardware(
    private val leftMotorId: Int,
    private val rightMotorId: Int,
    private val gearRatio: Double,
    private val leftMotorInverted: Boolean = false,
    private val rightMotorInverted: Boolean = true
) : ElevatorIO {

    // helper function to configure the motor
    private fun configureMotor(motorId: Int, isInverted: Boolean): TalonFX {
        return TalonFX(motorId).apply {
            val config = TalonFXConfiguration()
            
            // Configure feedback
            config.Feedback.SensorToMechanismRatio = gearRatio
            
            // Configure current limits
            // config.CurrentLimits.StatorCurrentLimit = 120.0
            // config.CurrentLimits.SupplyCurrentLimit = 40.0
            // config.CurrentLimits.StatorCurrentLimitEnable = true
            // config.CurrentLimits.SupplyCurrentLimitEnable = true
            // config.TorqueCurrent.withPeakForwardTorqueCurrent(120.0)
            // config.TorqueCurrent.withPeakReverseTorqueCurrent(-120.0)
            
            // Configure motor direction
            config.MotorOutput.Inverted = 
                if (isInverted) InvertedValue.CounterClockwise_Positive
                else InvertedValue.Clockwise_Positive
            
            // Set to brake mode
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake
            
            configurator.apply(config)
        }
    }

    private val leftMotor = configureMotor(leftMotorId, leftMotorInverted)
    private val rightMotor = configureMotor(rightMotorId, rightMotorInverted)

    // Status signals for monitoring
    private val leftPosition = leftMotor.position.clone()
    private val leftVelocity = leftMotor.velocity.clone()
    private val leftSupplyVolts = leftMotor.supplyVoltage.clone()
    private val leftMotorVolts = leftMotor.motorVoltage.clone()
    private val leftStatorCurrent = leftMotor.statorCurrent.clone()

    private val rightPosition = rightMotor.position.clone()
    private val rightVelocity = rightMotor.velocity.clone()
    private val rightSupplyVolts = rightMotor.supplyVoltage.clone()
    private val rightMotorVolts = rightMotor.motorVoltage.clone()
    private val rightStatorCurrent = rightMotor.statorCurrent.clone()

    override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
        // Refresh all status signals
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftSupplyVolts,
            leftMotorVolts,
            leftStatorCurrent,
            rightPosition,
            rightVelocity,
            rightSupplyVolts,
            rightMotorVolts,
            rightStatorCurrent
        )

        // Update left motor inputs
        inputs.leftMotorVoltage.mut_replace(leftMotorVolts.value)
        inputs.leftSupplyVoltage.mut_replace(leftSupplyVolts.value)
        inputs.leftAngularVelocity.mut_replace(leftVelocity.value)
        inputs.leftStatorCurrent.mut_replace(leftStatorCurrent.value)
        inputs.leftEncoderPosition.mut_replace(leftPosition.value)

        // Update right motor inputs
        inputs.rightMotorVoltage.mut_replace(rightMotorVolts.value)
        inputs.rightSupplyVoltage.mut_replace(rightSupplyVolts.value)
        inputs.rightAngularVelocity.mut_replace(rightVelocity.value)
        inputs.rightStatorCurrent.mut_replace(rightStatorCurrent.value)
        inputs.rightEncoderPosition.mut_replace(rightPosition.value)
    }

    override fun setLeftMotorVoltage(voltage: Voltage) {
        leftMotor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun setRightMotorVoltage(voltage: Voltage) {
        rightMotor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun stop() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }
}