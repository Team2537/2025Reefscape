package frc.robot.subsystems.climb

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage

class ClimbIOHardware (
    private val id: Int,
    private val isInverted: Boolean,
    private val gearing: Double,
    private val drumRadius: Meters,
    ): ClimberArmIO {
        private val motor: CANSparkMax = CanSparkMax (id, CanSparkLowLevel.MotorType.kBrushless).apply {
            inverted = isInverted
            encoder.angularVelocityConversionFactor = 1/gearing
            idleMode = CANSparkBase.IdleMode.kBrake
        }
    override fun updateInputs(inputs: ClimberArmIO.ClimberArmInputs) {
        inputs.angleFromAbsoluteEncoder.mut_replace()
        inputs.angularVelocity.mut_replace(
            motor.encoder.angularVelocity / 60) * (drumRadius into Meters) * 2 * Math.Pi,
        MetersPerSecond)
        inputs.supplyVoltage.mut_replace()
        inputs.motorVoltage.mut_replace()
        inputs.statorCurrent.mut_replace(Amps.of(motor.outputCurrent))
    }
    override fun setVoltage(voltage: Volts, isClosedLoop: Boolean {
        motor.setVoltage(voltage into Volts)
    }
    override fun setBrakeMode() {
    }
    override fun stop() {
        motor.stopMotor()
    }

}