package frc.robot.subsystems.climb

import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import kotlin.math.PI

class ClimbIONeo(
    private val id: Int,
    private val isInverted: Boolean,
    private val gearing: Double,
) : ClimbIO {
    private val brakeConfig = SparkMaxConfig().apply {
        inverted(isInverted)
        encoder.positionConversionFactor(1.0 / gearing)
        encoder.velocityConversionFactor(1.0 / gearing)
        idleMode(SparkBaseConfig.IdleMode.kBrake)
        smartCurrentLimit(30)
    }

    private val coastConfig = brakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

    private val motor = SparkMax(id, SparkLowLevel.MotorType.kBrushless).apply {
        configure(brakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: ClimbIO.ClimbArmInputs) {
        inputs.absoluteAngle.mut_replace(Rotations.of(motor.encoder.position))
        inputs.angularVelocity.mut_replace(
            RotationsPerSecond.of(motor.encoder.velocity / 60.0)
        )
        inputs.supplyVoltage.mut_replace(Volts.of(motor.busVoltage))
        inputs.motorVoltage.mut_replace(Volts.of(motor.appliedOutput * motor.busVoltage))
        inputs.statorCurrent.mut_replace(Amps.of(motor.outputCurrent))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun setBrakeMode(brakeEnabled: Boolean) {
        motor.configure(
            if(brakeEnabled) brakeConfig else coastConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    override fun stop() {
        motor.stopMotor()
    }
}