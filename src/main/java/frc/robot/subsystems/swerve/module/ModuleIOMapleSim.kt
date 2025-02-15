package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meter
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.amps
import lib.math.units.into
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController

class ModuleIOMapleSim(
    private val moduleSim: SwerveModuleSimulation,
    private val drivePidGains: PIDGains,
    private val driveFeedforwardGains: FeedforwardGains,
    private val turnPidGains: PIDGains,
    private val wheelRadius: Distance
) : ModuleIO {
    
    private val driveController = PIDController(
        drivePidGains.kP,
        drivePidGains.kI,
        drivePidGains.kD
    )
    
    private val turnController = PIDController(
        turnPidGains.kP,
        turnPidGains.kI,
        turnPidGains.kD
    ).apply { enableContinuousInput(-Math.PI, Math.PI) }
    
    private val driveFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        driveFeedforwardGains.kS,
        driveFeedforwardGains.kV,
        driveFeedforwardGains.kA
    )
    
    
    private val driveAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
    private val turnAppliedVoltage: MutVoltage = Volts.zero().mutableCopy()
    
    private var driveClosedLoop: Boolean = false
    
    private var driveFFVolts: Double = 0.0
    
    private var turnClosedLoop: Boolean = false
    
    private val driveMotor: GenericMotorController = moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(80.0.amps)
    private val turnMotor: GenericMotorController = moduleSim.useGenericControllerForSteer().withCurrentLimit(30.0.amps)
    
    /**
     * Update the passed in inputs with the current state of the module, using sensor data
     *
     * @param inputs The inputs to update
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        if (driveClosedLoop) {
            driveAppliedVoltage.mut_replace(
                driveFFVolts +
                  driveController.calculate(moduleSim.driveWheelFinalSpeed into RotationsPerSecond),
                Volts
            )
        } else {
            driveController.reset()
        }
        
        if (turnClosedLoop) {
            turnAppliedVoltage.mut_replace(
                turnController.calculate(moduleSim.steerAbsoluteFacing.rotations),
                Volts
            )
        } else {
            turnController.reset()
        }
        
        driveMotor.requestVoltage(driveAppliedVoltage)
        turnMotor.requestVoltage(turnAppliedVoltage)
        
        inputs.isDriveMotorConnected = true
        inputs.drivePositionAngular.mut_replace(moduleSim.driveWheelFinalPosition)
        inputs.drivePosition.mut_replace(
            (moduleSim.driveWheelFinalPosition into Radians) * (wheelRadius into Meters),
            Meter
        )
        inputs.driveVelocity.mut_replace(
            (moduleSim.driveWheelFinalSpeed into RotationsPerSecond) * (wheelRadius into Meters),
            MetersPerSecond
        )
        inputs.driveVelocityAngular.mut_replace(moduleSim.driveWheelFinalSpeed)
        
        inputs.driveAppliedVoltage.mut_replace(driveAppliedVoltage)

        inputs.isTurnMotorConnected = true
        inputs.isAbsoluteEncoderConnected = true
        
        inputs.absoluteTurnPosition = moduleSim.steerAbsoluteFacing
        inputs.turnPosition = moduleSim.steerAbsoluteFacing
        inputs.turnVelocity.mut_replace(moduleSim.steerAbsoluteEncoderSpeed)
        inputs.turnAppliedVoltage.mut_replace(turnAppliedVoltage)
    }
    
    /**
     * Send a voltage to the drive motor
     *
     * @param volts The voltage to send to the motor
     */
    override fun setDriveVoltage(volts: Voltage) {
        driveClosedLoop = false
        driveAppliedVoltage.mut_replace(volts)
    }
    
    /**
     * Send a voltage to the turn motor
     *
     * @param volts The voltage to send to the motor
     */
    override fun setTurnVoltage(volts: Voltage) {
        turnClosedLoop = false
        turnAppliedVoltage.mut_replace(volts)
    }
    
    /**
     * Send a position setpoint to the turn motor
     *
     * @param position The position to set the motor to
     */
    override fun setTurnPosition(position: Rotation2d) {
        turnClosedLoop = true
        turnController.setSetpoint(position.rotations)
    }
    
    /**
     * Send a velocity setpoint to the drive motor
     *
     * @param velocity The velocity to set the motor to
     */
    override fun setDriveVelocity(velocity: LinearVelocity) {
        val velocityRadPerSec = (velocity into MetersPerSecond) / (wheelRadius into Meters)
        driveClosedLoop = true
        driveFFVolts = driveFeedforward.calculate(velocityRadPerSec)
        driveController.setSetpoint(velocityRadPerSec)
    }
}