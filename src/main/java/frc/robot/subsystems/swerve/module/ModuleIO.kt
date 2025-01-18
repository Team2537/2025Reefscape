package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.units.measure.*
import lib.controllers.gains.PIDGains
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * IO Interface for defining universal methods for swerve modules
 *
 * This interface defines the methods that a swerve module must implement in order to be used in the
 * swerve drive
 */
interface ModuleIO {
    /**
     * The inputs for the module
     *
     * This class is used to store the current state of the module's sensors and motors
     */
    class ModuleInputs : LoggableInputs {
        var isDriveMotorConnected = false
        var isTurnMotorConnected = false
        var isAbsoluteEncoderConnected = false
        
        var driveVelocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()
        var drivePosition: MutDistance = Meters.zero().mutableCopy()
        var driveAppliedVoltage: MutVoltage = Volt.zero().mutableCopy()
        var driveStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var driveSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()
        var driveTorqueCurrent: MutCurrent = Amps.zero().mutableCopy()
        var driveTargetVelocityAngular: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var driveVelocityAngular: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        
        var turnPosition: Rotation2d = Rotation2d.kZero
        var absoluteTurnPosition: Rotation2d = Rotation2d.kZero
        
        var turnVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var turnAppliedVoltage: MutVoltage = Volt.zero().mutableCopy()
        var turnStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var turnSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()
        
        
        
        override fun toLog(table: LogTable) {
            table.put("isDriveMotorConnected", isDriveMotorConnected)
            table.put("isTurnMotorConnected", isTurnMotorConnected)
            table.put("isAbsoluteEncoderConnected", isAbsoluteEncoderConnected)
            
            table.put("driveVelocity", driveVelocity)
            table.put("drivePosition", drivePosition)
            table.put("driveAppliedVoltage", driveAppliedVoltage)
            table.put("driveStatorCurrent", driveStatorCurrent)
            table.put("driveSupplyCurrent", driveSupplyCurrent)
            table.put("driveTorqueCurrent", driveTorqueCurrent)
            table.put("driveTargetVelocityAngular", driveTargetVelocityAngular)
            table.put("driveVelocityAngular", driveVelocityAngular)
            
            table.put("turnVelocity", turnVelocity)
            table.put("absoluteTurnPosition", Rotation2d.struct, absoluteTurnPosition)
            
            table.put("turnPosition", Rotation2d.struct, turnPosition)
            table.put("turnAppliedVoltage", turnAppliedVoltage)
            table.put("turnStatorCurrent", turnStatorCurrent)
            table.put("turnSupplyCurrent", turnSupplyCurrent)
        }
        
        override fun fromLog(table: LogTable) {
            isDriveMotorConnected = table.get("isDriveMotorConnected", isDriveMotorConnected)
            isTurnMotorConnected = table.get("isTurnMotorConnected", isTurnMotorConnected)
            isAbsoluteEncoderConnected = table.get("isAbsoluteEncoderConnected", isAbsoluteEncoderConnected)
            
            driveVelocity.mut_replace(table.get("driveVelocity", driveVelocity))
            drivePosition.mut_replace(table.get("drivePosition", drivePosition))
            driveAppliedVoltage.mut_replace(table.get("driveAppliedVoltage", driveAppliedVoltage))
            driveStatorCurrent.mut_replace(table.get("driveStatorCurrent", driveStatorCurrent))
            driveSupplyCurrent.mut_replace(table.get("driveSupplyCurrent", driveSupplyCurrent))
            driveTorqueCurrent.mut_replace(table.get("driveTorqueCurrent", driveTorqueCurrent))
            driveTargetVelocityAngular.mut_replace(table.get("driveTargetVelocityAngular", driveTargetVelocityAngular))
            driveVelocityAngular.mut_replace(table.get("driveVelocityAngular", driveVelocityAngular))
            
            turnPosition = table.get("turnPosition", Rotation2d.struct, turnPosition)
            absoluteTurnPosition = table.get("absoluteTurnPosition", Rotation2d.struct, absoluteTurnPosition)
            
            turnVelocity.mut_replace(table.get("turnVelocity", turnVelocity))
            turnAppliedVoltage.mut_replace(table.get("turnAppliedVoltage", turnAppliedVoltage))
            turnStatorCurrent.mut_replace(table.get("turnStatorCurrent", turnStatorCurrent))
            turnSupplyCurrent.mut_replace(table.get("turnSupplyCurrent", turnSupplyCurrent))
        }
        
    }
    
    /**
     * Update the passed in inputs with the current state of the module, using sensor data
     *
     * @param inputs The inputs to update
     */
    fun updateInputs(inputs: ModuleInputs) {}
    
    /**
     * Send a voltage to the drive motor
     *
     * @param volts The voltage to send to the motor
     */
    fun setDriveVoltage(volts: Voltage) {}
    
    /**
     * Send a voltage to the turn motor
     *
     * @param volts The voltage to send to the motor
     */
    fun setTurnVoltage(volts: Voltage) {}
    
    /**
     * Send a position setpoint to the turn motor
     *
     * @param position The position to set the motor to
     */
    fun setTurnPosition(position: Rotation2d) {}
    
    /**
     * Send a velocity setpoint to the drive motor
     *
     * @param velocity The velocity to set the motor to
     */
    fun setDriveVelocity(velocity: LinearVelocity) {}
    
    /** Reset the encoder positions of the module */
    fun reset() {}
    
    /** Stop the module */
    fun stop() {}

    fun setTurnBrake(enabled: Boolean) {}
    fun setDriveBrake(enabled: Boolean) {}

    fun setDrivePID(gains: PIDGains) {}
    fun setSteerPID(gains: PIDGains) {}

    fun setDriveCurrent(current: Current) {}


}