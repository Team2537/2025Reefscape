package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.angularToLinearD
import lib.math.units.into

class ModuleIOSim(
    val driveFF: FeedforwardGains,
    val driveGains: PIDGains,
    driveMotorGearbox: DCMotor,
    val driveGearing: Double,
    val turnFF: FeedforwardGains,
    val turnGains: PIDGains,
    turnMotorGearbox: DCMotor,
    val turnGearing: Double,
    val wheelRadius: Distance
) : ModuleIO {
    val driveMotor: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            driveMotorGearbox,
            0.025,
            driveGearing
        ),
        driveMotorGearbox
    )
    
    val turnMotor: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            turnMotorGearbox,
            0.004,
            turnGearing
        ),
        turnMotorGearbox
    )
    
    val driveFeedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(driveFF.kS, driveFF.kV, driveFF.kA)
    
    val turnFeedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(turnFF.kS, turnFF.kV, turnFF.kA)
    
    val driveFeedback: PIDController =
        PIDController(driveGains.kP, driveGains.kI, driveGains.kD)
    
    val turnFeedback: PIDController =
        PIDController(turnGains.kP, turnGains.kI, turnGains.kD)
    
    /**
     * Update the passed in inputs with the current state of the module, using sensor data
     *
     * @param inputs The inputs to update
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        driveMotor.update(0.02)
        turnMotor.update(0.02)
        
        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true
        inputs.absoluteEncoderConnected = true
        
        inputs.drivePosition.mut_replace(
            driveMotor.angularPositionRad * wheelRadius.into(Meters),
            Meters
        )
        
        inputs.driveVelocity.mut_replace(
            driveMotor.angularVelocityRadPerSec * wheelRadius.into(Meters),
            MetersPerSecond
        )
        
        inputs.driveAppliedVoltage.mut_replace(driveMotor.inputVoltage, Volts)
        inputs.driveStatorCurrent.mut_replace(driveMotor.currentDrawAmps, Amps)
        inputs.driveSupplyCurrent.mut_replace(driveMotor.currentDrawAmps, Amps)
        
        inputs.turnPosition = Rotation2d.fromRadians(turnMotor.angularPositionRad)
        inputs.absoluteTurnPosition = inputs.turnPosition
        
        inputs.turnVelocity.mut_replace(turnMotor.angularVelocityRadPerSec, RadiansPerSecond)
        inputs.turnAppliedVoltage.mut_replace(turnMotor.inputVoltage, Volts)
        inputs.turnStatorCurrent.mut_replace(turnMotor.currentDrawAmps, Amps)
        inputs.turnSupplyCurrent.mut_replace(turnMotor.currentDrawAmps, Amps)
    }
    
    
    
    /**
     * Send a voltage to the drive motor
     *
     * @param volts The voltage to send to the motor
     */
    override fun setDriveVoltage(volts: Voltage) {
        driveMotor.inputVoltage = volts into Volts
    }
    
    /**
     * Send a voltage to the turn motor
     *
     * @param volts The voltage to send to the motor
     */
    override fun setTurnVoltage(volts: Voltage) {
        turnMotor.inputVoltage = volts into Volts
    }
    
    /**
     * Send a position setpoint to the turn motor
     *
     * @param position The position to set the motor to
     */
    override fun setTurnPosition(position: Rotation2d) {
        turnMotor.setAngle(position.radians)
    }
    
    /**
     * Send a velocity setpoint to the drive motor
     *
     * @param velocity The velocity to set the motor to
     */
    override fun setDriveVelocity(velocity: LinearVelocity) {
        val velocityRadPerSec = (velocity into MetersPerSecond) / wheelRadius.into(Meters)
        val feedforward = driveFeedforward.calculate(velocityRadPerSec)
        val output = driveFeedback.calculate(
            driveMotor.angularVelocityRPM / 60.0,
            velocityRadPerSec
        )

        driveMotor.setAngularVelocity(velocityRadPerSec)

//        driveMotor.inputVoltage = (output + feedforward)
    }
    
    /** Reset the encoder positions of the module */
    override fun reset() {
        driveMotor.setState(0.0, driveMotor.angularVelocityRadPerSec)
    }
    
    /** Stop the module */
    override fun stop() {
        driveMotor.inputVoltage = 0.0
        turnMotor.inputVoltage = 0.0
    }
}