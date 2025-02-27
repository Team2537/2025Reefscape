package frc.robot.subsystems.superstructure.gripper

import com.reduxrobotics.sensors.canandcolor.Canandcolor
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*

class GripperIONeo (
    private val id: Int,
    private val sensorID: Int,
    private val isInverted: Boolean,
    private val gearing: Double,
) : GripperIO {
    private val coastConfig = SparkMaxConfig().apply {
        inverted(isInverted)
        encoder.positionConversionFactor(1.0 / gearing)
        encoder.velocityConversionFactor(1.0 / gearing)
        idleMode(SparkBaseConfig.IdleMode.kCoast)
        smartCurrentLimit(30)
    }
    private val motor = SparkMax(id, SparkLowLevel.MotorType.kBrushless).apply {
        configure(coastConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    private val canandcolor = Canandcolor(sensorID)

    private val coralSensorThreshold : Double = 0.25

    override fun updateInputs(inputs: GripperIO.GripperInputs) {
        inputs.gripperPosition.mut_replace(Rotations.of(motor.encoder.position))
        inputs.gripperVelocity.mut_replace(
            RotationsPerSecond.of(motor.encoder.velocity / 60.0)
        )
        inputs.gripperAppliedVoltage.mut_replace(Volts.of(motor.appliedOutput * motor.busVoltage))
        inputs.gripperStatorCurrent.mut_replace(Amps.of(motor.outputCurrent))

        inputs.isHoldingCoral = canandcolor.proximity < coralSensorThreshold
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun setBrakeMode(enabled: Boolean) {
        val brakeConfig = SparkMaxConfig()
        brakeConfig.idleMode(if(enabled) SparkBaseConfig.IdleMode.kBrake else SparkBaseConfig.IdleMode.kCoast)
        motor.configure(brakeConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }
}
