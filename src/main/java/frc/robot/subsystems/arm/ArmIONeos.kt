package frc.robot.subsystems.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import lib.controllers.gains.ControllerGains
import lib.math.units.amps
import lib.math.units.into

/**
 * ArmIO for real robot.
 *
 * @constructor Constructs a new ArmIO with the given id, gains, and limits
 * @param id The id of the motor
 * @param gains The controller gains of the arm
 * @param kG The gravitational gain for an arm; [gains][ControllerGains]
 * does not have this, so it must be provided separately
 * @param gearing The gearing between the encoder and output
 * @param encoderId The id of the absolute encoder
 * @param encoderOffset The offset applied to the absolute encoder
 * @param statorCurrentLimit The limit of the stator current. Defaults
 * to `120A`. Passing `null` will use the TalonFX default
 * @param supplyCurrentLimit The limit of the supply current. Defaults
 * to `40A`. Passing `null` will use the TalonFX default
 */
class ArmIONeos(
    id: Int,
    gains: ControllerGains,
    kG: Double, // Gravity gain; not included in ControllerGains
    gearing: Double,

    encoderId: Int,
    encoderOffset: Rotation2d,

    // null for either of these uses the default values of a talon
    statorCurrentLimit: Current? = 120.0.amps,
    supplyCurrentLimit: Current? = 40.0.amps,
) : ArmIO {
    private val motor: TalonFX = TalonFX(id).apply {
        val config: TalonFXConfiguration = TalonFXConfiguration()

        config.Feedback.SensorToMechanismRatio = gearing

        config.Slot0.kP = gains.pid.kP
        config.Slot0.kI = gains.pid.kI
        config.Slot0.kD = gains.pid.kD

        config.Slot0.kS = gains.feedforward.kS
        config.Slot0.kV = gains.feedforward.kV
        config.Slot0.kA = gains.feedforward.kA
        config.Slot0.kG = kG

        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimitEnable = true

        if(statorCurrentLimit !== null) {
            config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit into Amps
        }

        if(supplyCurrentLimit !== null) {
            config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit into Amps
        }

        config.TorqueCurrent.withPeakForwardTorqueCurrent(120.0)
        config.TorqueCurrent.withPeakReverseTorqueCurrent(-120.0)

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake

        configurator.apply(config)
    }

    private val absoluteEncoder: CANcoder = CANcoder(encoderId).apply {
        val config = CANcoderConfiguration()

        config.MagnetSensor.MagnetOffset = encoderOffset.rotations

        configurator.apply(config)
    }

    private val velocity: StatusSignal<AngularVelocity> = motor.velocity.clone()
    private val relativePosition: StatusSignal<Angle> = motor.position.clone()
    private val absolutePosition: StatusSignal<Angle> = absoluteEncoder.position.clone()
    private val motorVoltage: StatusSignal<Voltage> = motor.motorVoltage.clone()
    private val supplyVoltage: StatusSignal<Voltage> = motor.supplyVoltage.clone()
    private val statorCurrent: StatusSignal<Current> = motor.statorCurrent.clone()

    /**
     * Updates the given inputs with data from the arm systems.
     *
     * @param inputs The inputs to update.
     */
    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        BaseStatusSignal.refreshAll(
            velocity,
            relativePosition,
            absolutePosition,
            motorVoltage,
            supplyVoltage,
            statorCurrent,
        )

        inputs.motorVelocity.mut_replace(velocity.value)
        inputs.motorRelativePosition.mut_replace(relativePosition.value)
        inputs.motorAbsolutePosition.mut_replace(absolutePosition.value)
        inputs.motorVoltage.mut_replace(motorVoltage.value)
        inputs.motorSupplyVoltage.mut_replace(supplyVoltage.value)
        inputs.motorStatorCurrent.mut_replace(statorCurrent.value)
    }

    /**
     * Gives the arm a setpoint angle. The given angle is an angle of
     * elevation (relative to the horizontal)
     *
     * @param angle The desired angle of elevation.
     */
    override fun setAngle(angle: Angle) {
        motor.setPosition(angle into Rotations)
    }

    /**
     * Sets the motor to run at a specific voltage.
     *
     * @param voltage The desired voltage.
     */
    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage into Volts)
    }

    /**
     * Stops the motor.
     */
    override fun stop() {
        motor.stopMotor()
    }
}