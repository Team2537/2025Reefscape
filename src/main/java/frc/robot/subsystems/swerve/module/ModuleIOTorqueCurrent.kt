package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
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

class ModuleIOTorqueCurrent(
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
            if(driveInverted) InvertedValue.CounterClockwise_Positive
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
    
    private val turnMotor = SparkMax(turnID, SparkLowLevel.MotorType.kBrushless).apply {
        val config = SparkMaxConfig().apply {
            closedLoop.pid(
                turnPID.kP,
                turnPID.kI,
                turnPID.kD,
            )
            
            closedLoop.positionWrappingEnabled(true)
            closedLoop.positionWrappingInputRange(-0.5, 0.5)
            
            inverted(turnInverted)
            encoder.positionConversionFactor(1/turnGearing)
            encoder.velocityConversionFactor(60/turnGearing)
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            smartCurrentLimit(30)
        }
        
        configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }
    
    private val absEncoder: CANcoder = CANcoder(encoderID).apply {
        val config = CANcoderConfiguration()
        
        config.MagnetSensor.MagnetOffset = encoderOffset.rotations
        
        
        configurator.apply(config)
    }
    
    val absoluteTurnPosition = absEncoder.absolutePosition.clone()
    
    val openLoopDriveRequest: VoltageOut = VoltageOut(0.0)
    val openLoopTorqueRequest: TorqueCurrentFOC = TorqueCurrentFOC(0.0)
    val closedLoopDriveRequest: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    
    init {
        turnMotor.encoder.setPosition(absoluteTurnPosition.valueAsDouble)
    }
    
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
        
        inputs.isTurnMotorConnected = true
        
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
            driveStatorCurrent.value)
        
        inputs.driveSupplyCurrent.mut_replace(
            driveSupplyCurrent.value)
        
        inputs.driveTorqueCurrent.mut_replace(
            driveTorqueCurrent.value)
        
        inputs.turnPosition = Rotation2d.fromRotations(turnMotor.encoder.position)
        
        inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.valueAsDouble)
        
        inputs.turnVelocity.mut_replace(turnMotor.encoder.velocity, RotationsPerSecond)
        
        inputs.turnAppliedVoltage.mut_replace(turnMotor.appliedOutput * turnMotor.busVoltage, Volts)
        
        inputs.turnStatorCurrent.mut_replace(turnMotor.outputCurrent, Amps)
        
        
        
        
    }
    
    override fun setDriveVoltage(volts: Voltage) {
        driveMotor.setControl(openLoopDriveRequest.withOutput(volts))
    }
    
    override fun setTurnVoltage(volts: Voltage) {
        turnMotor.setVoltage(volts into Volts)
    }
    
    override fun setTurnPosition(position: Rotation2d) {
        turnMotor.closedLoopController.setReference(
            position.rotations,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            turnFF.kS,
        )
    }
    
    override fun setDriveVelocity(velocity: LinearVelocity) {
        driveMotor.setControl(
            closedLoopDriveRequest.withVelocity(
                (velocity into MetersPerSecond) / (wheelRadius into Meters) measuredIn RadiansPerSecond,
            ),
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
        
        config.MotorOutput.NeutralMode = if(enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
    }
    
    override fun setTurnBrake(enabled: Boolean) {
        val config = SparkMaxConfig()
        
        config.idleMode(if(enabled) SparkBaseConfig.IdleMode.kBrake else SparkBaseConfig.IdleMode.kCoast)
        
        turnMotor.configure(
            config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        println("Setting brake to $enabled")
    }
    
    override fun setDrivePID(gains: PIDGains) {
        val config = Slot0Configs()
        driveMotor.configurator.refresh(config)
        driveMotor
            .configurator
            .apply(config
                .withKP(gains.kP)
                .withKI(gains.kI)
                .withKD(gains.kD)
            )
    }
    
    override fun setSteerPID(gains: PIDGains) {
    
    }
    
    override fun setDriveCurrent(current: Current) {
        driveMotor.setControl(openLoopTorqueRequest.withOutput(current))
    }
}