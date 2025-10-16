package frc.robot.subsystems.superstructure.manipulator

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.ManipulatorConstants
// import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIO.ManipulatorInputs
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import lib.math.controllers.gains.PIDGains
import lib.math.controllers.gains.FeedforwardGains
import frc.robot.subsystems.superstructure.manipulator.*
import java.util.function.Supplier

class Manipulator : SubsystemBase() {
    public val io: ManipulatorIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> ManipulatorIOKraken(
            leftRollerMotorID = Constants.ManipulatorConstants.LEFT_ROLLER_MOTOR_ID,
            rightRollerMotorID = Constants.ManipulatorConstants.RIGHT_ROLLER_MOTOR_ID,
            canandcolorID = Constants.ManipulatorConstants.CANANDCOLOR_ID,
            leftRollerInverted = Constants.ManipulatorConstants.LEFT_ROLLER_INVERTED,
            rightRollerInverted = Constants.ManipulatorConstants.RIGHT_ROLLER_INVERTED,
            rollerGearing = Constants.ManipulatorConstants.ROLLER_GEARING,
        )

        RobotType.Mode.SIMULATION -> ManipulatorIOSim(
            rollerGearing = Constants.ManipulatorConstants.ROLLER_GEARING,
            rollerMoi = Constants.ManipulatorConstants.SIM_ROLLER_MOI,
        )

        RobotType.Mode.REPLAY -> object : ManipulatorIO {}
    }
    
    public val inputs: ManipulatorInputs = ManipulatorInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)
    }

    fun getSpinRollersInCommand(): Command {
        return runOnce {
            io.setRollerVoltage(Constants.ManipulatorConstants.ROLLER_IN_VOLTAGE)
        }
    }

    fun getSpinRollersInSlowCommand(): Command {
        return runOnce {
            io.setRollerVoltage(Constants.ManipulatorConstants.ROLLER_IN_VOLTAGE * 0.25)
        }
    }

    fun getSpinRollersOutCommand(): Command {
        return runOnce {
            io.setRollerVoltage(Constants.ManipulatorConstants.ROLLER_OUT_VOLTAGE)
        }
    }

    fun getStopRollersCommand(): Command {
        return runOnce {
            io.stopRoller()
        }
    }

    fun isDetectingGamePiece(): Trigger {
        return Trigger { inputs.coralDistance < Constants.ManipulatorConstants.DETECTION_DISTANCE_THRESHOLD }
    }
}
