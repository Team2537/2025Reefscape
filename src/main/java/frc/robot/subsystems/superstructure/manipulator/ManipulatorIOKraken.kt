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
    pivotMotorID: Int,
    canandcolorID: Int,
    rollerMotorID: Int,
    pivotInverted: Boolean,
    rollerInverted: Boolean,
    pivotGearing: Double,
    rollerGearing: Double,
    pivotPIDGains: PIDGains,
    pivotFFGains: FeedforwardGains,
    pivotKG: Double,
    pivotVelocityLimit: AngularVelocity,
    pivotAccelerationLimit: AngularAcceleration,
    pivotJerkLimit: Velocity<AngularAccelerationUnit> = RotationsPerSecondPerSecond.per(Second).of(0.0)
) : ManipulatorIO {

    private fun configureMotor(
        motorId: Int,
        isInverted: Boolean,
        pidGains: PIDGains,
        ffGains: FeedforwardGains,
        kG: Double,
        velocityLimit: AngularVelocity,
        accelerationLimit: AngularAcceleration,
        jerkLimit: Velocity<AngularAccelerationUnit>,
        gearing: Double,
        isRoller: Boolean = false
    ): TalonFX {
        return TalonFX(motorId, "canivore").apply {
            val config = TalonFXConfiguration()

            config.MotorOutput.Inverted =
                if (isInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive

            config.MotorOutput.NeutralMode = if (isRoller) NeutralModeValue.Coast else NeutralModeValue.Brake

            if (!isRoller) {
                config.Slot0.kP = pidGains.kP
                config.Slot0.kI = pidGains.kI
                config.Slot0.kD = pidGains.kD
                config.Slot0.kV = ffGains.kV
                config.Slot0.kA = ffGains.kA
                config.Slot0.kS = ffGains.kS
                config.Slot0.kG = kG

                config.Slot0.GravityType = GravityTypeValue.Arm_Cosine

                config.Feedback.SensorToMechanismRatio = gearing

                config.ClosedLoopGeneral.ContinuousWrap = true

                config.MotionMagic.withMotionMagicCruiseVelocity(velocityLimit)
                config.MotionMagic.withMotionMagicAcceleration(accelerationLimit)
                config.MotionMagic.withMotionMagicJerk(jerkLimit)
            }

            config.CurrentLimits.StatorCurrentLimit = 60.0
            config.CurrentLimits.StatorCurrentLimitEnable = true

            configurator.apply(config)
        }
    }

    private val pivotMotor = configureMotor(
        pivotMotorID,
        pivotInverted,
        pivotPIDGains,
        pivotFFGains,
        pivotKG,
        pivotVelocityLimit,
        pivotAccelerationLimit,
        pivotJerkLimit,
        pivotGearing
    ).apply {
        // start at retracted position
        setPosition(Constants.ManipulatorConstants.PIVOT_START_ANGLE)
    }

    private val pivotPosition = pivotMotor.position.clone()
    private val pivotVelocity = pivotMotor.velocity.clone()
    private val pivotAppliedVoltage = pivotMotor.motorVoltage.clone()
    private val pivotStatorCurrent = pivotMotor.statorCurrent.clone()

    private val motionMagicRequest = MotionMagicVoltage(0.0)
    private val pivotVoltageRequest = VoltageOut(0.0)

    private val canandcolor = Canandcolor(canandcolorID)

    // kraken x44 for rolling motors
    private val rollerMotor = configureMotor(
        rollerMotorID,
        rollerInverted,
        pidGains = PIDGains(), // These will be ignored for roller motor
        ffGains = FeedforwardGains(),  // These will be ignored for roller motor
        kG = 0.0,       // These will be ignored for roller motor
        velocityLimit = RotationsPerSecond.of(0.0), // These will be ignored for roller motor
        accelerationLimit = RotationsPerSecondPerSecond.of(0.0), // These will be ignored for roller motor
        jerkLimit = RotationsPerSecondPerSecond.per(Second).of(0.0), // These will be ignored for roller motor
        gearing = rollerGearing,
        true // isRoller
    )

    private val rollerVoltageRequest = VoltageOut(0.0)

    private val rollerVelocity = rollerMotor.velocity.clone()
    private val rollerAppliedVoltage = rollerMotor.motorVoltage.clone()
    private val rollerStatorCurrent = rollerMotor.statorCurrent.clone()

    override fun updateInputs(inputs: ManipulatorIO.ManipulatorInputs) {
        BaseStatusSignal.refreshAll(
            pivotPosition, pivotVelocity, pivotAppliedVoltage, pivotStatorCurrent,
            rollerVelocity, rollerAppliedVoltage, rollerStatorCurrent
        )

        // TODO: convert getProximity to meters
        inputs.coralDistance.mut_replace(Meters.of(0.0))

        inputs.pivotAngularPosition.mut_replace(pivotPosition.value)
        inputs.pivotAngularVelocity.mut_replace(pivotVelocity.value)
        inputs.pivotAppliedVoltage.mut_replace(pivotAppliedVoltage.value)
        inputs.pivotStatorCurrent.mut_replace(pivotStatorCurrent.value)

        inputs.rollerAngularVelocity.mut_replace(rollerVelocity.value)
        inputs.rollerAppliedVoltage.mut_replace(rollerAppliedVoltage.value)
        inputs.rollerStatorCurrent.mut_replace(rollerStatorCurrent.value)
    }

    override fun setPivotTargetAngle(angle: Angle) {
        pivotMotor.setControl(motionMagicRequest.withPosition(angle))
    }

    override fun setPivotVoltage(voltage: Voltage) {
        pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage))
    }

    override fun setRollerVoltage(voltage: Voltage) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage))
    }

    override fun setPivotBrakeMode(brake: Boolean) {
        pivotMotor.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun stopRoller() {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(Volts.of(0.0)))
    }
} 