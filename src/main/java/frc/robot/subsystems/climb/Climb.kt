package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.climb.ClimbIO.ClimbArmInputs
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.units.Units.*
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands
import lib.math.units.into

class Climb : SubsystemBase() {
    private val io: ClimbIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> ClimbIONeo(
            Constants.ClimbConstants.CLIMB_MOTOR_ID,
            Constants.ClimbConstants.IS_CLIMB_MOTOR_INVERTED,
            Constants.ClimbConstants.CLIMB_GEAR_RATIO
        )

        RobotType.Mode.SIMULATION -> ClimbIOSim(
            DCMotor.getNEO(1),
            Constants.ClimbConstants.CLIMB_GEAR_RATIO,
            Constants.ClimbConstants.MOI,
            Constants.ClimbConstants.ARM_LENGTH
        )

        RobotType.Mode.REPLAY -> object : ClimbIO {}
    }

    private val inputs: ClimbArmInputs = ClimbArmInputs()

    init {
        // set brake mode to true so climb arm doesn't move around in the internals of the robot while driving around
        io.setBrakeMode(true)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("Climb", inputs)

        // i'm not sure what to do for the Logger.processOutputs() stuff
    }

    fun getExtendCommand(): Command {
        // move the arm to the extension angle with bang bang control
        return Commands.sequence(
            runOnce {
                io.setBrakeMode(false)
            },
            run {
                val targetAngle = Constants.ClimbConstants.EXTENDED_ANGLE
                val currentAngle = inputs.absoluteAngle.plus(Constants.ClimbConstants.ABSOLUTE_OFFSET)
                
                // Apply voltage based on whether we need to move up or down
                if (currentAngle > targetAngle) {
                    io.setVoltage(-Constants.ClimbConstants.EXTEND_VOLTAGE) // Move up
                } else if (currentAngle < targetAngle) {
                    io.setVoltage(Constants.ClimbConstants.EXTEND_VOLTAGE) // Move down
                } else {
                    io.stop()
                }
            }.until( {
                // End command when we're close enough to target angle
                kotlin.math.abs((inputs.absoluteAngle into Degrees) - 
                    (Constants.ClimbConstants.EXTENDED_ANGLE into Degrees)) < (Constants.ClimbConstants.EXTEND_ANGLE_TOLERANCE into Degrees)
            } ),
            runOnce {
                io.stop()
                io.setBrakeMode(true)
            }
        )
    }

    // start rotating the arm until the current spikes and velocity drops to 0, indicating we have hit a hard stop
    // then stop the motor and set the brake mode to true
    fun getRetractCommand(): Command {
        return Commands.sequence(
            runOnce {
                io.setBrakeMode(false)
            },
            run {
                io.setVoltage(-Constants.ClimbConstants.RETRACT_VOLTAGE)
            },
            Commands.waitUntil {
                // End command when we detect a hard stop (high current and near-zero velocity)
                val currentSpike = inputs.statorCurrent > Constants.ClimbConstants.CURRENT_SPIKE_THRESHOLD
                val velocityNearZero = kotlin.math.abs(inputs.angularVelocity into RotationsPerSecond) < Constants.ClimbConstants.VELOCITY_ZERO_THRESHOLD into RotationsPerSecond
                currentSpike && velocityNearZero
            },
            runOnce {
                io.stop()
                io.setBrakeMode(true)
            }
        )
    }
}