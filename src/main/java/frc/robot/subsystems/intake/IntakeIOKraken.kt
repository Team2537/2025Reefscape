package frc.robot.subsystems.intake

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

class IntakeIOKraken(
    leftMotorID: Int,
    rightMotorID: Int,
    canandcolorID: Int,
    rollerMotorID: Int,
    leftInverted: Boolean,
    rightInverted: Boolean,
    rollerInverted: Boolean,
    gearing: Double,
    pidGains: PIDGains,
    ffGains: FeedforwardGains,
    kG: Double,
    velocityLimit: AngularVelocity,
    accelerationLimit: AngularAcceleration,
    jerkLimit: Velocity<AngularAccelerationUnit> = RotationsPerSecondPerSecond.per(Second).of(0.0)
) : IntakeIO {

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

    private val leftMotor = configureMotor(
        leftMotorID,
        leftInverted,
        pidGains,
        ffGains,
        kG,
        velocityLimit,
        accelerationLimit,
        jerkLimit,
        gearing
    ).apply {
        // start at retracted position
        setPosition(Constants.IntakeConstants.PIVOT_RETRACTED_ANGLE)
    }

    private val rightMotor = configureMotor(
        rightMotorID,
        rightInverted,
        pidGains,
        ffGains,
        kG,
        velocityLimit,
        accelerationLimit,
        jerkLimit,
        gearing
    ).apply {
        setPosition(Constants.IntakeConstants.PIVOT_RETRACTED_ANGLE)
        setControl(Follower(leftMotorID, true)) // true means invert the left motor
    }
    
    private val leftPosition = leftMotor.position.clone()
    private val leftVelocity = leftMotor.velocity.clone()
    private val leftAppliedVoltage = leftMotor.motorVoltage.clone()
    private val leftStatorCurrent = leftMotor.statorCurrent.clone()

    private val rightPosition = rightMotor.position.clone()
    private val rightVelocity = rightMotor.velocity.clone()
    private val rightAppliedVoltage = rightMotor.motorVoltage.clone()
    private val rightStatorCurrent = rightMotor.statorCurrent.clone()
    private var pivotSetAngle = Constants.IntakeConstants.PIVOT_RETRACTED_ANGLE

    private val motionMagicRequest = MotionMagicVoltage(0.0)
    private val pivotVoltageRequest = VoltageOut(0.0)

    private val canandcolor = Canandcolor(canandcolorID)

    // kraken x44 for rolling motors
    private val rollerMotor = configureMotor(
        rollerMotorID,
        rollerInverted,
        pidGains, // These will be ignored for roller motor
        ffGains,  // These will be ignored for roller motor
        kG,       // These will be ignored for roller motor
        velocityLimit, // These will be ignored for roller motor
        accelerationLimit, // These will be ignored for roller motor
        jerkLimit, // These will be ignored for roller motor
        gearing,
        true // isRoller
    )

    private val rollerVoltageRequest = VoltageOut(0.0)

    private val rollerPosition = rollerMotor.position.clone()
    private val rollerVelocity = rollerMotor.velocity.clone()
    private val rollerAppliedVoltage = rollerMotor.motorVoltage.clone()
    private val rollerStatorCurrent = rollerMotor.statorCurrent.clone()

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        BaseStatusSignal.refreshAll(
            leftPosition, leftVelocity, leftAppliedVoltage, leftStatorCurrent,
            rightPosition, rightVelocity, rightAppliedVoltage, rightStatorCurrent,
            rollerVelocity, rollerAppliedVoltage, rollerStatorCurrent
        )

        // TODO: convert getProximity to meters
        inputs.coralDistance.mut_replace(Meters.of(0.0))

        inputs.pivotLeftPosition.mut_replace(leftPosition.value)
        inputs.pivotLeftVelocity.mut_replace(leftVelocity.value)
        inputs.pivotLeftAppliedVoltage.mut_replace(leftAppliedVoltage.value)
        inputs.pivotLeftStatorCurrent.mut_replace(leftStatorCurrent.value)

        inputs.pivotRightPosition.mut_replace(rightPosition.value)
        inputs.pivotRightVelocity.mut_replace(rightVelocity.value)
        inputs.pivotRightAppliedVoltage.mut_replace(rightAppliedVoltage.value)
        inputs.pivotRightStatorCurrent.mut_replace(rightStatorCurrent.value)
        inputs.pivotSetAngle.mut_replace(pivotSetAngle)

        inputs.rollerVelocity.mut_replace(rollerVelocity.value)
        inputs.rollerAppliedVoltage.mut_replace(rollerAppliedVoltage.value)
        inputs.rollerStatorCurrent.mut_replace(rollerStatorCurrent.value)
    }

    override fun setPivotAngle(angle: Angle) {
        leftMotor.setControl(motionMagicRequest.withPosition(angle))
        pivotSetAngle = angle
    }

    override fun setPivotVoltage(voltage: Voltage) {
        leftMotor.setControl(pivotVoltageRequest.withOutput(voltage))
    }

    override fun setRollerVoltage(voltage: Voltage) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage))
    }

    override fun setPivotBrakeMode(brake: Boolean) {
        leftMotor.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun stopRoller() {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(Volts.of(0.0)))
        // may need to brake too, will try out this first
    }
    
}