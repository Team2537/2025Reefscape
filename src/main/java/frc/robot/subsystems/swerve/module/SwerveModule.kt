package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import frc.robot.RobotType
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.measuredIn
import lib.math.units.metersPerSecond
import lib.math.vector

/**
 * Represents a swerve module on the robot.
 *
 * @param io The IO implementation for the module.
 * @param modulePosition The position of the module relative to the center of the robot.
 */
class SwerveModule(
    val driveID: Int,
    val turnID: Int,
    val encoderID: Int,
    val invertDrive: Boolean,
    val invertTurn: Boolean,
    val encoderOffset: Rotation2d,
    val modulePosition: Translation2d
) {
    private val io: ModuleIO = when(RobotType.type){
        RobotType.Type.SIMULATION_BOT -> ModuleIOSim(
            FeedforwardGains(),
            PIDGains(),
            DCMotor.getKrakenX60Foc(1),
            6.75,
            FeedforwardGains(),
            PIDGains(),
            DCMotor.getNEO(1),
            150/7.0,
            2.0 measuredIn Inches
        )
        
        RobotType.Type.SWERVE_TEST_BOT -> TODO()
        RobotType.Type.COMPETITION_BOT -> TODO()
        RobotType.Type.REPLAY -> TODO()
    }
    
    /** The object that holds the inputs for the module. */
    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()
    
    /** The state of the module. */
    val state: SwerveModuleState
        get() = SwerveModuleState(
            inputs.driveVelocity,
            inputs.absoluteTurnPosition
        )
    
    /** The position of the module. */
    val position: SwerveModulePosition
        get() = SwerveModulePosition(
            inputs.drivePosition,
            inputs.absoluteTurnPosition
        )
    
    /**
     * Vector pointing in the direction that the module would be facing if it was contributing 100% of its velocity
     * to the robot rotating counter-clockwise.
     */
    val positiveRotVec = (modulePosition.angle + Rotation2d.fromDegrees(90.0)).vector
    
    /** Updates the inputs for the module. */
    fun periodic() { io.updateInputs(inputs) }
    
    /**
     * Applies the desired state to the module.
     *
     * @param desiredState The desired state for the module.
     */
    fun applyState(desiredState: SwerveModuleState) {
        desiredState.optimize(inputs.absoluteTurnPosition)
        desiredState.cosineScale(inputs.absoluteTurnPosition)
        
        io.setTurnPosition(desiredState.angle)
        io.setDriveVelocity(desiredState.speedMetersPerSecond measuredIn Units.MetersPerSecond)
    }
}