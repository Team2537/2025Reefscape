package frc.robot.subsystems.superstructure.manipulator.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains

class ManipulatorWristKraken(
    motorID: Int,
    pidGains: PIDGains,
    ffGains: FeedforwardGains,
    inverted: Boolean,
    gearing: Double,
    cruiseVelocity: AngularVelocity,
    acceleration: AngularAcceleration
): ManipulatorWristIO {
    private val motor: TalonFX = TalonFX(motorID).apply {
        val config = TalonFXConfiguration()

        config.Slot0.kP = pidGains.kP
        config.Slot0.kI = pidGains.kI
        config.Slot0.kD = pidGains.kD
        config.Slot0.kS = ffGains.kS
        config.Slot0.kV = ffGains.kV
        config.Slot0.kA = ffGains.kA
        config.Slot0.kG = 0.45
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine

        config.MotionMagic
            .withMotionMagicCruiseVelocity(cruiseVelocity)
            .withMotionMagicAcceleration(acceleration)

        config.MotorOutput.Inverted = if(inverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake

        config.Feedback.SensorToMechanismRatio = gearing

        config.CurrentLimits.StatorCurrentLimit = 80.0
        config.CurrentLimits.SupplyCurrentLimit = 40.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimitEnable = true

        configurator.apply(config)

        setPosition(Units.degreesToRotations(10.0))
    }

    private val position = motor.position.clone()
    private val velocity = motor.velocity.clone()
    private val current = motor.statorCurrent.clone()
    private val voltage = motor.motorVoltage.clone()

    private val openLoopRequest: VoltageOut = VoltageOut(0.0)
    private val closedLoopRequest: MotionMagicVoltage = MotionMagicVoltage(0.0)

    override fun updateInputs(inputs: ManipulatorWristIO.ManipulatorWristInputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
            position, velocity, current, voltage
        ).isOK

        inputs.angle.mut_replace(position.value)
        inputs.velocity.mut_replace(velocity.value)
        inputs.appliedVoltage.mut_replace(voltage.value)
        inputs.statorCurrent.mut_replace(current.value)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(openLoopRequest.withOutput(voltage))
    }

    override fun setTargetPosition(targetAngle: Angle) {
        motor.setControl(closedLoopRequest.withPosition(targetAngle))
    }
}