package frc.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
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

    private val pid = PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD)
    private val ff = ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA)

    private var positionControlEnabled: Boolean = false
    private var targetAngle: Angle = ArmConstants.MIN_ANGLE

    init {
        val commonRatio = ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO

        val leftCfg = TalonFXConfiguration().apply {
            MotorOutput.Inverted = if (ArmConstants.LEFT_INVERTED) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Feedback.SensorToMechanismRatio = commonRatio
            CurrentLimits.StatorCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
        }
        left.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration().apply {
            MotorOutput.Inverted = if (ArmConstants.RIGHT_INVERTED) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Feedback.SensorToMechanismRatio = commonRatio
            CurrentLimits.StatorCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
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

        if (positionControlEnabled) {
            val currentAngleRad = leftPosition.value into Radians
            val pidOut = pid.calculate(currentAngleRad, targetAngle.baseUnitMagnitude())
            val ffOut = ff.calculate(currentAngleRad, 0.0)
            val voltsCmd = pidOut + ffOut
            val volts = Volts.of(voltsCmd)
            left.setControl(voltageRequest.withOutput(volts))
            right.setControl(voltageRequest.withOutput(volts))
        }
    }

    override fun setVoltage(voltage: Voltage) {
        positionControlEnabled = false
        left.setControl(voltageRequest.withOutput(voltage))
        right.setControl(voltageRequest.withOutput(voltage))
    }

    override fun setTargetAngle(angle: Angle) {
        positionControlEnabled = true
        targetAngle = angle
        pid.reset()
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


