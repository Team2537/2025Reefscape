package frc.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains

class ArmIOKraken(
    motorID: Int,
    inverted: Boolean,
    gearing: Double,
    pidGains: PIDGains,
    ffGains: FeedforwardGains,
    kG: Double,
    velocityLimit: AngularVelocity,
    accelerationLimit: AngularAcceleration,
    jerkLimit: Velocity<AngularAccelerationUnit> = RotationsPerSecondPerSecond.per(Second).of(0.0)
) : ArmIO {
    private val motor = TalonFX(motorID).apply {
        val config = TalonFXConfiguration()

        config.MotorOutput.Inverted =
            if (inverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive

        config.Slot0.kP = pidGains.kP
        config.Slot0.kI = pidGains.kI
        config.Slot0.kD = pidGains.kD
        config.Slot0.kV = ffGains.kV
        config.Slot0.kA = ffGains.kA
        config.Slot0.kS = ffGains.kS
        config.Slot0.kG = kG

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine

        config.Feedback.SensorToMechanismRatio = gearing

        config.ClosedLoopGeneral.ContinuousWrap = true

        config.CurrentLimits.StatorCurrentLimit = 40.0
        config.CurrentLimits.StatorCurrentLimitEnable = true

        config.MotionMagic.withMotionMagicCruiseVelocity(velocityLimit)
        config.MotionMagic.withMotionMagicAcceleration(accelerationLimit)
        config.MotionMagic.withMotionMagicJerk(jerkLimit)

        configurator.apply(config)

        setPosition(Degrees.of(90.0))
    }

    private val position = motor.position.clone()
    private val velocity = motor.velocity.clone()
    private val appliedVoltage = motor.motorVoltage.clone()
    private val statorCurrent = motor.statorCurrent.clone()
    private val supplyVoltage = motor.supplyVoltage.clone()

    private val motionMagicRequest = MotionMagicVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVoltage, statorCurrent, supplyVoltage
        )

        inputs.motorRelativePosition.mut_replace(position.value)
        inputs.motorVelocity.mut_replace(velocity.value)
        inputs.motorVoltage.mut_replace(appliedVoltage.value)
        inputs.motorStatorCurrent.mut_replace(statorCurrent.value)
        inputs.motorSupplyVoltage.mut_replace(supplyVoltage.value)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(motionMagicRequest.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun stop() {
        motor.stopMotor()
    }
}