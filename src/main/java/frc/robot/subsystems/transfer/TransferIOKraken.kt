package frc.robot.subsystems.transfer

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.subsystems.transfer.TransferIO.TransferInputs
import edu.wpi.first.units.measure.Voltage


class TransferIOKraken(
    private val id: Int,
    private val isInverted: Boolean,
    private val gearing: Double,
) : TransferIO {
    // kraken x44 motor
    val motor: TalonFX = TalonFX(id).apply {
        val config = TalonFXConfiguration()

        config.MotorOutput.Inverted =
            if (isInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake
    }

    private val motorVoltageRequest = VoltageOut(0.0)

    private val velocity = motor.velocity.clone()
    private val appliedVoltage = motor.motorVoltage.clone()
    private val statorCurrent = motor.statorCurrent.clone()

    override fun updateInputs(inputs: TransferInputs) {
        BaseStatusSignal.refreshAll(
            velocity, appliedVoltage, statorCurrent
        )

        inputs.velocity.mut_replace(velocity.value)
        inputs.appliedVoltage.mut_replace(appliedVoltage.value)
        inputs.statorCurrent.mut_replace(statorCurrent.value)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(motorVoltageRequest.withOutput(voltage))
    }

    override fun setBrakeMode(brake: Boolean) {
        motor.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
}

