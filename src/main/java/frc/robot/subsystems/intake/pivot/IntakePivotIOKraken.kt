package frc.robot.subsystems.intake.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

/**
 * Implementation of PivotIO for a Kraken motor controlling a pivot mechanism with hard stops.
 * The pivot moves between two physical limits and uses relative encoder position.
 */
class IntakePivotIOKraken(
    private val motorID: Int,
    private val gearing: Double,
    private val isInverted: Boolean,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains,
    private val kG: Double,
    private val motionMagicAcceleration: AngularAcceleration,
    private val motionMagicCruiseVelocity: AngularVelocity,
    private val motionMagicJerk: Velocity<AngularAccelerationUnit> = RotationsPerSecondPerSecond.per(Second).of(0.0),
) : IntakePivotIO {

    // Initialize Kraken motor with configuration
    private val motor = TalonFX(motorID).apply {
        val config = TalonFXConfiguration()

        // Configure feedback sensor
        config.Feedback.SensorToMechanismRatio = gearing

        // Configure PID gains
        config.Slot0.kP = pidGains.kP
        config.Slot0.kI = pidGains.kI
        config.Slot0.kD = pidGains.kD
        config.Slot0.kV = ffGains.kV
        config.Slot0.kA = ffGains.kA
        config.Slot0.kS = ffGains.kS
        config.Slot0.kG = kG

        // Configure motion magic parameters
        config.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration)
        config.MotionMagic.withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        config.MotionMagic.withMotionMagicJerk(motionMagicJerk)

        // Configure current limits
        config.CurrentLimits.StatorCurrentLimit = 80.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimit = 40.0
        config.CurrentLimits.SupplyCurrentLimitEnable = true

        // Configure motor direction and neutral mode
        config.MotorOutput.withInverted(
            if (isInverted) InvertedValue.CounterClockwise_Positive
            else InvertedValue.Clockwise_Positive
        )
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)

        configurator.apply(config)
    }

    // Create status signal objects for monitoring motor state
    private val position = motor.position.clone()
    private val velocity = motor.velocity.clone()
    private val motorVoltage = motor.motorVoltage.clone()
    private val statorCurrent = motor.statorCurrent.clone()

    // Control requests for voltage and position control
    private val voltageRequest = VoltageOut(0.0)
    private val positionRequest = MotionMagicVoltage(0.0)

    override fun updateInputs(inputs: IntakePivotIO.IntakePivotInputs) {
        // Refresh all status signals
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            motorVoltage,
            statorCurrent
        )

        // Update input values with current motor state
        inputs.position.mut_replace(position.valueAsDouble, Rotations)
        inputs.velocity.mut_replace(velocity.valueAsDouble, RotationsPerSecond)
        inputs.appliedVoltage.mut_replace(motorVoltage.valueAsDouble, Volts)
        inputs.statorCurrent.mut_replace(statorCurrent.valueAsDouble, Amps)
    }

    override fun setVoltage(volts: Voltage) {
        motor.setControl(voltageRequest.withOutput(volts into Volts))
    }

    override fun setTargetAngle(angle: Angle) {
        motor.setControl(
            positionRequest
                .withPosition(angle into Rotations)
                .withSlot(0)
        )
    }
}
