package frc.robot.subsystems.intake.roller
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage


class IntakeRollerIONeo (
    private val id: Int,
    private val isInverted: Boolean,
    private val gearing: Double

) : IntakeRollerIO {
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

    override fun updateInputs(inputs: IntakeRollerIO.IntakeRollerInputs) {
        inputs.position.mut_replace(Rotations.of(motor.encoder.position))
        inputs.velocity.mut_replace(
                RPM.of(motor.encoder.velocity)
        )
        inputs.appliedVoltage.mut_replace(Volts.of(motor.appliedOutput * motor.busVoltage))
        inputs.statorCurrent.mut_replace(Amps.of(motor.outputCurrent))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun setBrakeMode(brake: Boolean) {
        val config = SparkMaxConfig().apply {
            if (brake) {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
            } else {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
            }
        }

        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }
}
