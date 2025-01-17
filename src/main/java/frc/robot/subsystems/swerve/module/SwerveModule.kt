package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.RobotType
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.measuredIn
import lib.math.vector
import lib.util.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

/**
 * Represents a swerve module on the robot.
 *
 * @param driveID The ID of the drive motor.
 * @param turnID The ID of the turn motor.
 * @param encoderID The ID of the encoder.
 * @param invertDrive Whether the drive motor is inverted.
 * @param invertTurn Whether the turn motor is inverted.
 * @param encoderOffset The offset of the encoder.
 * @param modulePosition The position of the module.
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

    private val io: ModuleIO = when (RobotType.mode) {
        RobotType.Mode.SIMULATION -> ModuleIOSim(
            FeedforwardGains(
                kV = 12/97.0
            ),
            PIDGains(
                kP = 0.01
            ),
            DCMotor.getKrakenX60Foc(1),
            6.75,
            FeedforwardGains(),
            PIDGains(),
            DCMotor.getNEO(1),
            150 / 7.0,
            2.0 measuredIn Inches
        )
        RobotType.Mode.REAL -> ModuleIOReal(
            driveID,
            6.75,
            invertDrive,
            FeedforwardGains(
                kS = 1.1531,
            ),
            PIDGains(kP = 0.005),
            turnID,
            150 / 7.0,
            turnInverted = invertTurn,
            FeedforwardGains(),
            PIDGains(kP = 5.0,),
            encoderID,
            encoderOffset,
            wheelRadius = 2.0 measuredIn Inches
        )
        else -> object : ModuleIO {}
    }
    
    /** The index of the module in the array of modules. Calculated with math */
    val index: Int = (driveID - 1) / 3

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
    
    /** The desired state of the module. */
    var desiredState: SwerveModuleState = SwerveModuleState()

    /**
     * Vector pointing in the direction that the module would be facing if it was contributing 100% of its velocity
     * to the robot rotating counter-clockwise.
     */
    val positiveRotationVector = (modulePosition.angle + Rotation2d.fromDegrees(90.0)).vector

    /** Updates the inputs for the module. */
    fun periodic() {
        io.updateInputs(inputs)
        
        Logger.recordOutput("modules/$index/driveError", inputs.driveVelocity.minus(desiredState.speedMetersPerSecond measuredIn Units.MetersPerSecond))
        Logger.recordOutput("modules/$index/turnError", inputs.absoluteTurnPosition.minus(desiredState.angle))
    }

    /**
     * Applies the desired state to the module.
     *
     * @param desiredState The desired state for the module.
     */
    fun applyState(desiredState: SwerveModuleState) {
        this.desiredState = desiredState
        
        desiredState.optimize(inputs.absoluteTurnPosition)
        desiredState.cosineScale(inputs.absoluteTurnPosition)

        io.setTurnPosition(desiredState.angle)
        io.setDriveVelocity(desiredState.speedMetersPerSecond measuredIn Units.MetersPerSecond)
    }

    fun characterizeVoltage(volts: Voltage) {
        io.setTurnPosition(Rotation2d())
        io.setDriveVoltage(volts)
    }

    fun characterizeCurrent(current: Current){
        io.setTurnPosition(Rotation2d())
        io.setDriveCurrent(current)
    }
}