package frc.robot.subsystems.superstructure.manipulator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import com.reduxrobotics.sensors.canandcolor.Canandcolor
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.controls.Follower
import frc.robot.Constants

class ManipulatorIOKraken(
    leftRollerMotorID: Int,
    rightRollerMotorID: Int,
    canandcolorID: Int,
    leftRollerInverted: Boolean,
    rightRollerInverted: Boolean,
    rollerGearing: Double,
) : ManipulatorIO {

    private fun configureMotor(
        motorId: Int,
        isInverted: Boolean,
        gearing: Double,
        isRoller: Boolean = true
    ): TalonFX {
        return TalonFX(motorId, "canivore").apply {
            val config = TalonFXConfiguration()

            config.MotorOutput.Inverted =
                if (isInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive

            config.MotorOutput.NeutralMode = if (isRoller) NeutralModeValue.Coast else NeutralModeValue.Brake

            config.Feedback.SensorToMechanismRatio = gearing

            config.CurrentLimits.StatorCurrentLimit = 60.0
            config.CurrentLimits.StatorCurrentLimitEnable = true

            configurator.apply(config)
        }
    }

    private val canandcolor = Canandcolor(canandcolorID)

    // Two Kraken X44 for rollers
    private val leftRollerMotor = configureMotor(
        leftRollerMotorID,
        leftRollerInverted,
        gearing = rollerGearing,
        isRoller = true
    )

    private val rightRollerMotor = configureMotor(
        rightRollerMotorID,
        rightRollerInverted,
        gearing = rollerGearing,
        isRoller = true
    )

    private val leftRollerVoltageRequest = VoltageOut(0.0)
    private val rightRollerVoltageRequest = VoltageOut(0.0)

    private val leftRollerVelocity = leftRollerMotor.velocity.clone()
    private val leftRollerAppliedVoltage = leftRollerMotor.motorVoltage.clone()
    private val leftRollerStatorCurrent = leftRollerMotor.statorCurrent.clone()

    private val rightRollerVelocity = rightRollerMotor.velocity.clone()
    private val rightRollerAppliedVoltage = rightRollerMotor.motorVoltage.clone()
    private val rightRollerStatorCurrent = rightRollerMotor.statorCurrent.clone()

    override fun updateInputs(inputs: ManipulatorIO.ManipulatorInputs) {
        BaseStatusSignal.refreshAll(
            leftRollerVelocity, leftRollerAppliedVoltage, leftRollerStatorCurrent,
            rightRollerVelocity, rightRollerAppliedVoltage, rightRollerStatorCurrent
        )

        inputs.coralDistance.mut_replace(Constants.ManipulatorConstants.CANANDCOLOR_SCALING_FACTOR * canandcolor.getProximity())

        inputs.leftRollerAngularVelocity.mut_replace(leftRollerVelocity.value)
        inputs.leftRollerAppliedVoltage.mut_replace(leftRollerAppliedVoltage.value)
        inputs.leftRollerStatorCurrent.mut_replace(leftRollerStatorCurrent.value)

        inputs.rightRollerAngularVelocity.mut_replace(rightRollerVelocity.value)
        inputs.rightRollerAppliedVoltage.mut_replace(rightRollerAppliedVoltage.value)
        inputs.rightRollerStatorCurrent.mut_replace(rightRollerStatorCurrent.value)
    }

    override fun setRollerVoltage(voltage: Voltage) {
        leftRollerMotor.setControl(leftRollerVoltageRequest.withOutput(voltage))
        rightRollerMotor.setControl(rightRollerVoltageRequest.withOutput(voltage))
    }

    override fun setLeftRightRollerVoltages(leftVoltage: Voltage, rightVoltage: Voltage) {
        leftRollerMotor.setControl(leftRollerVoltageRequest.withOutput(leftVoltage))
        rightRollerMotor.setControl(rightRollerVoltageRequest.withOutput(rightVoltage))
    }

    override fun stopRoller() {
        leftRollerMotor.setControl(leftRollerVoltageRequest.withOutput(Volts.of(0.0)))
        rightRollerMotor.setControl(rightRollerVoltageRequest.withOutput(Volts.of(0.0)))
    }
} 