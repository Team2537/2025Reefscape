package frc.robot.subsystems.intake.roller

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.intake.roller.IntakeRollerIO.IntakeRollerInputs
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands

class IntakeRoller : SubsystemBase() {
    private val io: IntakeRollerIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> IntakeRollerIONeo(
            Constants.IntakeConstants.RollerConstants.MOTOR_ID,
            Constants.IntakeConstants.RollerConstants.IS_MOTOR_INVERTED,
            Constants.IntakeConstants.RollerConstants.GEARING
        )

        RobotType.Mode.SIMULATION -> IntakeRollerIOSim(
            DCMotor.getNEO(1),
            Constants.IntakeConstants.RollerConstants.GEARING,
            Constants.IntakeConstants.RollerConstants.MOI
        )

        RobotType.Mode.REPLAY -> object : IntakeRollerIO {}
    }
    
    private val inputs: IntakeRollerInputs = IntakeRollerInputs()

    init{
        io.setBrakeMode(true)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("IntakeRoller", inputs)

        // TODO: add logging for outputs
    }

    fun getRollCommand(): Command {
        return Commands.sequence(
            runOnce {
                io.setBrakeMode(false)
            },
            run {
                io.setVoltage(Constants.IntakeConstants.RollerConstants.ROLLER_VOLTAGE)
            },
            Commands.waitUntil {
                inputs.velocity < Constants.IntakeConstants.RollerConstants.MINIMUM_ROLLER_VELOCITY
            },
            runOnce {
                io.setVoltage(Volts.zero())
                io.setBrakeMode(true)
            }
        )
    }
}