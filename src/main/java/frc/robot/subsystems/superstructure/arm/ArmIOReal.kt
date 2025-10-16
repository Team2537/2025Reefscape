package frc.robot.subsystems.superstructure.arm

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import frc.robot.Constants.ArmConstants

class ArmIOReal : ArmIO {
    private val left = SparkMax(ArmConstants.LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val right = SparkMax(ArmConstants.RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val positionFactor = (2.0 * Math.PI) / (ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO)
    private val velocityFactor = positionFactor / 60.0

    private val leftBrakeConfig = SparkMaxConfig().apply {
        inverted(ArmConstants.LEFT_INVERTED)
        idleMode(SparkBaseConfig.IdleMode.kBrake)
        smartCurrentLimit(50)
        encoder.positionConversionFactor(positionFactor)
        encoder.velocityConversionFactor(velocityFactor)
    }
    private val leftCoastConfig = leftBrakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

    private val rightBrakeConfig = SparkMaxConfig().apply {
        inverted(ArmConstants.RIGHT_INVERTED)
        idleMode(SparkBaseConfig.IdleMode.kBrake)
        smartCurrentLimit(50)
        encoder.positionConversionFactor(positionFactor)
        encoder.velocityConversionFactor(velocityFactor)
    }
    private val rightCoastConfig = rightBrakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

    private val pid = PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD)
    private val ff = ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA)

    private var positionControlEnabled: Boolean = false
    private var targetAngle: Angle = ArmConstants.MIN_ANGLE

    init {
        left.configure(leftBrakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        right.configure(rightBrakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        inputs.isLeftMotorConnected = true
        inputs.isRightMotorConnected = true
        inputs.angle.mut_replace(left.encoder.position, Radians)
        inputs.angularVelocity.mut_replace(left.encoder.velocity, RadiansPerSecond)
        inputs.appliedVoltage.mut_replace(Volts.of(left.appliedOutput * left.busVoltage))
        inputs.leftStatorCurrent.mut_replace(Amps.of(left.outputCurrent))
        inputs.rightStatorCurrent.mut_replace(Amps.of(right.outputCurrent))

        if (positionControlEnabled) {
            val currentAngleRad = left.encoder.position
            val pidOut = pid.calculate(currentAngleRad, targetAngle.baseUnitMagnitude())
            val ffOut = ff.calculate(currentAngleRad, 0.0)
            val voltsCmd = pidOut + ffOut
            left.setVoltage(voltsCmd)
            right.setVoltage(voltsCmd)
        }
    }

    override fun setVoltage(voltage: Voltage) {
        positionControlEnabled = false
        left.setVoltage(voltage.baseUnitMagnitude())
        right.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun setTargetAngle(angle: Angle) {
        positionControlEnabled = true
        targetAngle = angle
        pid.reset()
    }

    override fun setBrakeMode(brake: Boolean) {
        left.configure(if (brake) leftBrakeConfig else leftCoastConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        right.configure(if (brake) rightBrakeConfig else rightCoastConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }

    override fun stop() {
        left.stopMotor()
        right.stopMotor()
    }
}


