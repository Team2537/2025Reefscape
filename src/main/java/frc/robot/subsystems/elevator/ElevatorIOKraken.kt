package frc.robot.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

class ElevatorIOKraken(
    private val leftMotorId: Int,
    private val rightMotorId: Int,
    private val gearRatio: Double,
    // the effective radius of the sprocket on the chain, 
    // used to determine the distance the elevator carriage travels per rotation of the motor
    private val drumRadius: Distance,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains,
    private val kG: Double,
    private val motionMagicAcceleration: LinearAcceleration, // meters per second squared
    private val motionMagicCruiseVelocity: LinearVelocity, // meters per second
    private val motionMagicJerk: Double, // meters per second cubed
    // it is very important that one of these is true and the other is false, or else the motors will fight each other
    private val leftMotorInverted: Boolean = false,
    private val rightMotorInverted: Boolean = true
) : ElevatorIO {

    // helper function to configure the motor
    private fun configureMotor(motorId: Int, isInverted: Boolean): TalonFX {
        return TalonFX(motorId).apply {
            val config = TalonFXConfiguration()
            
            // Configure feedback
            config.Feedback.SensorToMechanismRatio = gearRatio
            
            // Configure PID and FF gains
            config.Slot0.kP = pidGains.kP
            config.Slot0.kI = pidGains.kI
            config.Slot0.kD = pidGains.kD
            config.Slot0.kV = ffGains.kV
            config.Slot0.kA = ffGains.kA
            config.Slot0.kS = ffGains.kS
            config.Slot0.kG = kG
            
            // Configure motion magic parameters
            config.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration.baseUnitMagnitude())
            config.MotionMagic.withMotionMagicCruiseVelocity(motionMagicCruiseVelocity.baseUnitMagnitude())
            config.MotionMagic.withMotionMagicJerk(motionMagicJerk)
            
            // Configure current limits
            config.CurrentLimits.StatorCurrentLimit = 120.0
            config.CurrentLimits.SupplyCurrentLimit = 40.0
            config.CurrentLimits.StatorCurrentLimitEnable = true
            config.CurrentLimits.SupplyCurrentLimitEnable = true
            config.TorqueCurrent.withPeakForwardTorqueCurrent(120.0)
            config.TorqueCurrent.withPeakReverseTorqueCurrent(-120.0)
            
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

    // Control requests
    private val voltageRequest = VoltageOut(0.0)
    private val motionMagicRequest = MotionMagicVoltage(0.0)

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

        // Update carriage position and velocity
        val rotationsToMeters = drumRadius.baseUnitMagnitude() * 2.0 * Math.PI
        inputs.carriageHeight.mut_replace(leftPosition.valueAsDouble * rotationsToMeters, Meters)
        inputs.carriageVelocity.mut_replace(leftVelocity.valueAsDouble * rotationsToMeters, MetersPerSecond)
    }

    override fun setElevatorHeightTarget(height: Distance) {
        val rotationsToMeters = drumRadius.baseUnitMagnitude() * 2.0 * Math.PI
        val targetRotations = height.baseUnitMagnitude() / rotationsToMeters
        
        leftMotor.setControl(
            motionMagicRequest
                .withPosition(targetRotations)
                .withSlot(0)
        )
        rightMotor.setControl(
            motionMagicRequest
                .withPosition(targetRotations)
                .withSlot(0)
        )
    }

    override fun stop() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }

    override fun setElevatorVoltage(volts: Voltage) {
        leftMotor.setControl(voltageRequest.withOutput(volts.baseUnitMagnitude()))
        rightMotor.setControl(voltageRequest.withOutput(volts.baseUnitMagnitude()))
    }
}