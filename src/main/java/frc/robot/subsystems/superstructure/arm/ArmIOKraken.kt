package frc.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import frc.robot.Constants.ArmConstants
import lib.math.units.into

class ArmIOKraken : ArmIO {
    private val left = TalonFX(ArmConstants.LEFT_MOTOR_ID)
    private val right = TalonFX(ArmConstants.RIGHT_MOTOR_ID)

    private val leftPosition = left.position.clone()
    private val leftVelocity = left.velocity.clone()
    private val leftAppliedVoltage = left.motorVoltage.clone()
    private val leftStatorCurrent = left.statorCurrent.clone()
    private val rightStatorCurrent = right.statorCurrent.clone()

    private val voltageRequest = VoltageOut(0.0)
    private val motionMagicRequest = MotionMagicVoltage(0.0)

    init {
        val commonRatio = ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO

        val leftCfg = TalonFXConfiguration().apply {
            MotorOutput.Inverted = if (ArmConstants.LEFT_INVERTED) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Feedback.SensorToMechanismRatio = commonRatio
            CurrentLimits.StatorCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true

            // Closed-loop gains and feedforward for Motion Magic (tune as needed)
            Slot0.kP = ArmConstants.KP
            Slot0.kI = ArmConstants.KI
            Slot0.kD = ArmConstants.KD
            Slot0.kS = ArmConstants.KS
            Slot0.kG = ArmConstants.KG
            Slot0.kV = ArmConstants.KV
            Slot0.kA = ArmConstants.KA
            Slot0.GravityType = GravityTypeValue.Arm_Cosine

            // Motion Magic trapezoidal profile settings (conservative starting points)
            MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2.0))
            MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10.0))
        }
        left.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration().apply {
            MotorOutput.Inverted = if (ArmConstants.RIGHT_INVERTED) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Feedback.SensorToMechanismRatio = commonRatio
            CurrentLimits.StatorCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true

            // Mirror left configs for consistency when running closed-loop on both motors
            Slot0.kP = ArmConstants.KP
            Slot0.kI = ArmConstants.KI
            Slot0.kD = ArmConstants.KD
            Slot0.kS = ArmConstants.KS
            Slot0.kG = ArmConstants.KG
            Slot0.kV = ArmConstants.KV
            Slot0.kA = ArmConstants.KA
            Slot0.GravityType = GravityTypeValue.Arm_Cosine

            MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2.0))
            MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10.0))
        }
        right.configurator.apply(rightCfg)
    }

    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        BaseStatusSignal.refreshAll(
            leftPosition, leftVelocity, leftAppliedVoltage, leftStatorCurrent, rightStatorCurrent
        )

        inputs.isLeftMotorConnected = true
        inputs.isRightMotorConnected = true
        inputs.angle.mut_replace(leftPosition.value)
        inputs.angularVelocity.mut_replace(leftVelocity.value)
        inputs.appliedVoltage.mut_replace(leftAppliedVoltage.value)
        inputs.leftStatorCurrent.mut_replace(leftStatorCurrent.value)
        inputs.rightStatorCurrent.mut_replace(rightStatorCurrent.value)

    }

    override fun setVoltage(voltage: Voltage) {
        left.setControl(voltageRequest.withOutput(voltage))
        right.setControl(voltageRequest.withOutput(voltage))
    }

    override fun setTargetAngle(angle: Angle) {
        // Convert commanded angle to mechanism rotations and send Motion Magic position setpoints
        val targetAngleRotations: Angle = Rotations.of(angle into Rotations)
        left.setControl(motionMagicRequest.withPosition(targetAngleRotations))
        right.setControl(motionMagicRequest.withPosition(targetAngleRotations))
    }

    override fun setBrakeMode(brake: Boolean) {
        val leftCfg = TalonFXConfiguration()
        left.configurator.refresh(leftCfg)
        leftCfg.MotorOutput.NeutralMode = if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast
        left.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration()
        right.configurator.refresh(rightCfg)
        rightCfg.MotorOutput.NeutralMode = if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast
        right.configurator.apply(rightCfg)
    }

    override fun stop() {
        left.setControl(voltageRequest.withOutput(Volts.of(0.0)))
        right.setControl(voltageRequest.withOutput(Volts.of(0.0)))
    }
}


