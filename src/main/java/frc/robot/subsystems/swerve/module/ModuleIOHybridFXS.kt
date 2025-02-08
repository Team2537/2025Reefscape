package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.TalonFXS
import com.ctre.phoenix6.signals.*
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into
import lib.math.units.measuredIn

class ModuleIOHybridFXS(
    val driveID: Int,
    val driveGearing: Double,
    val driveInverted: Boolean,
    val driveFF: FeedforwardGains,
    val drivePID: PIDGains,

    val turnID: Int,
    val turnGearing: Double,
    val turnInverted: Boolean,
    val turnFF: FeedforwardGains,
    val turnPID: PIDGains,

    val encoderID: Int,
    val encoderOffset: Rotation2d,

    val wheelRadius: Distance
) : ModuleIO {
    private val driveMotor: TalonFX = TalonFX(driveID).apply {
        val config: TalonFXConfiguration = TalonFXConfiguration()

        config.Feedback.SensorToMechanismRatio = driveGearing

        config.Slot0.kP = drivePID.kP
        config.Slot0.kI = drivePID.kI
        config.Slot0.kD = drivePID.kD
        config.Slot0.kV = driveFF.kV
        config.Slot0.kA = driveFF.kA
        config.Slot0.kS = driveFF.kS

        config.CurrentLimits.StatorCurrentLimit = 120.0
        config.CurrentLimits.SupplyCurrentLimit = 40.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimitEnable = true
        config.TorqueCurrent.withPeakForwardTorqueCurrent(120.0)
        config.TorqueCurrent.withPeakReverseTorqueCurrent(-120.0)

        config.MotorOutput.Inverted =
            if (driveInverted) InvertedValue.CounterClockwise_Positive
            else InvertedValue.Clockwise_Positive

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake

        configurator.apply(config)
    }

    private val drivePosition = driveMotor.position.clone()
    private val driveVelocity = driveMotor.velocity.clone()
    private val driveSupplyVolts = driveMotor.supplyVoltage.clone()
    private val driveMotorVolts = driveMotor.motorVoltage.clone()
    private val driveStatorCurrent = driveMotor.statorCurrent.clone()
    private val driveSupplyCurrent = driveMotor.supplyCurrent.clone()
    private val driveTorqueCurrent = driveMotor.torqueCurrent.clone()

    private val turnMotor = TalonFXS(turnID).apply {
        val config: TalonFXSConfiguration = TalonFXSConfiguration()

        config.ExternalFeedback.RotorToSensorRatio = turnGearing
        config.ExternalFeedback.FeedbackRemoteSensorID = encoderID
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder

        config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST
        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled

        config.ClosedLoopGeneral.ContinuousWrap = true
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake

        config.Slot0.kP = drivePID.kP
        config.Slot0.kI = drivePID.kI
        config.Slot0.kD = drivePID.kD
        config.Slot0.kV = driveFF.kV
        config.Slot0.kA = driveFF.kA
        config.Slot0.kS = driveFF.kS

        configurator.apply(config)
    }

    private val turnPosition = turnMotor.position.clone()
    private val turnVelocity = turnMotor.velocity.clone()
    private val turnAppliedVoltage = turnMotor.motorVoltage.clone()
    private val turnSupplyCurrent = turnMotor.supplyCurrent.clone()
    private val turnStatorCurrent = turnMotor.statorCurrent.clone()

    private val absEncoder: CANcoder = CANcoder(encoderID).apply {
        val config = CANcoderConfiguration()

        config.MagnetSensor.MagnetOffset = encoderOffset.rotations


        configurator.apply(config)
    }

    val absoluteTurnPosition = absEncoder.absolutePosition.clone()

    val openLoopDriveRequest: VoltageOut = VoltageOut(0.0)
    val openLoopTorqueRequest: TorqueCurrentFOC = TorqueCurrentFOC(0.0)
    val closedLoopDriveRequest: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)

    val closedLoopTurnRequest: MotionMagicExpoVoltage = MotionMagicExpoVoltage(0.0)

    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.isDriveMotorConnected = BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveSupplyVolts,
            driveMotorVolts,
            driveStatorCurrent,
            driveSupplyCurrent,
            driveTorqueCurrent
        ).isOK

        inputs.isTurnMotorConnected = BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnSupplyCurrent,
            turnAppliedVoltage,
            turnStatorCurrent
        ).isOK

        inputs.isAbsoluteEncoderConnected = BaseStatusSignal.refreshAll(
            absoluteTurnPosition
        ).isOK

        inputs.driveVelocity.mut_replace(
            (driveVelocity.value into RadiansPerSecond) * (wheelRadius into Meters),
            MetersPerSecond
        )

        inputs.drivePosition.mut_replace(
            (drivePosition.value into Radians) * (wheelRadius into Meters),
            Meters
        )

        inputs.drivePositionAngular.mut_replace(drivePosition.value)

        inputs.driveAppliedVoltage.mut_replace(
            driveMotorVolts.value
        )

        inputs.driveStatorCurrent.mut_replace(
            driveStatorCurrent.value
        )

        inputs.driveSupplyCurrent.mut_replace(
            driveSupplyCurrent.value
        )

        inputs.driveTorqueCurrent.mut_replace(
            driveTorqueCurrent.value
        )

        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.valueAsDouble)

        inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.valueAsDouble)

        inputs.turnVelocity.mut_replace(turnVelocity.value)

        inputs.turnAppliedVoltage.mut_replace(turnAppliedVoltage.value)

        inputs.turnStatorCurrent.mut_replace(turnStatorCurrent.value)
        inputs.turnSupplyCurrent.mut_replace(turnSupplyCurrent.value)
    }

    override fun setDriveVoltage(volts: Voltage) {
        driveMotor.setControl(openLoopDriveRequest.withOutput(volts))
    }

    override fun setTurnVoltage(volts: Voltage) {
        turnMotor.setVoltage(volts into Volts)
    }

    override fun setTurnPosition(position: Rotation2d) {
        turnMotor.setControl(closedLoopTurnRequest.withPosition(position.rotations))
    }

    override fun setDriveVelocity(velocity: LinearVelocity) {
        setDriveVelocity(velocity, Amps.zero())
    }

    /**
     * Send a velocity setpoint to the drive motor
     *
     * @param velocity The velocity to set the motor to
     * @param torqueCurrentFF The feedforward torque current to apply
     */
    override fun setDriveVelocity(velocity: LinearVelocity, torqueCurrentFF: Current) {
        driveMotor.setControl(
            closedLoopDriveRequest.withVelocity(
                (velocity into MetersPerSecond) / (wheelRadius into Meters) measuredIn RadiansPerSecond,
            ).withFeedForward(torqueCurrentFF),
        )
    }

    override fun reset() {
        driveMotor.setPosition(0.0)
    }

    override fun stop() {
        driveMotor.stopMotor()
        turnMotor.stopMotor()
    }

    override fun setDriveBrake(enabled: Boolean) {
        val config = TalonFXConfiguration()
        driveMotor.configurator.refresh(config)

        config.MotorOutput.NeutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
    }

    override fun setTurnBrake(enabled: Boolean) {
        val config = MotorOutputConfigs()
        turnMotor.configurator.refresh(config)
        turnMotor.configurator.apply(config.withNeutralMode(if(enabled) NeutralModeValue.Brake else NeutralModeValue.Coast))
    }

    override fun setDrivePID(gains: PIDGains) {
        val config = Slot0Configs()
        driveMotor.configurator.refresh(config)
        driveMotor
            .configurator
            .apply(
                config
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
            )
    }

    override fun setSteerPID(gains: PIDGains) {
        val config = Slot0Configs()
        turnMotor.configurator.refresh(config)
        turnMotor
            .configurator
            .apply(
                config
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD))
    }

    override fun setDriveCurrent(current: Current) {
        driveMotor.setControl(openLoopTorqueRequest.withOutput(current))
    }
}