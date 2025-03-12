package frc.robot.subsystems.superstructure.manipulator.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains

class ManipulatorWristKraken(
    motorID: Int,
    pidGains: PIDGains,
    ffGains: FeedforwardGains,
    inverted: Boolean,
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

        config.MotionMagic
            .withMotionMagicCruiseVelocity(cruiseVelocity)
            .withMotionMagicAcceleration(acceleration)

        config.MotorOutput.Inverted = if(inverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake

        config.CurrentLimits.StatorCurrentLimit = 80.0
        config.CurrentLimits.SupplyCurrentLimit = 40.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimitEnable = true
    }
}