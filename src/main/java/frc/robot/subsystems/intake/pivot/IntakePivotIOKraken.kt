package frc.robot.subsystems.intake.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import lib.math.units.into

/**
 * Implementation of PivotIO for a Kraken motor controlling a pivot mechanism with hard stops.
 * The pivot moves between two physical limits and uses relative encoder position.
 */
class PivotIOKraken(
    private val motorID: Int,
    private val gearing: Double,
    private val isInverted: Boolean,
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val kS: Double,
    private val kV: Double,
    private val kA: Double,
    private val motionMagicAcceleration: Double, // rotations per second squared
    private val motionMagicCruiseVelocity: Double, // rotations per second
    private val motionMagicJerk: Double // rotations per second cubed
) : IntakePivotIO {

    // Initialize Kraken motor with configuration
    private val motor = TalonFX(motorID).apply {
        val config = TalonFXConfiguration()

        // Configure feedback sensor
        config.Feedback.SensorToMechanismRatio = gearing

        // Configure PID gains
        config.Slot0.kP = kP
        config.Slot0.kI = kI
        config.Slot0.kD = kD
        config.Slot0.kV = kV
        config.Slot0.kA = kA
        config.Slot0.kS = kS

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

    override fun updateInputs(inputs: PivotIO.PivotInputs) {
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
