package frc.robot.subsystems.transfer

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.transfer.TransferIO.TransferInputs
import frc.robot.subsystems.transfer.TransferIOSim
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger

class Transfer : SubsystemBase() {
    public val io: TransferIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> TransferIOKraken(
            id = Constants.TransferConstants.MOTOR_ID,
            isInverted = Constants.TransferConstants.IS_MOTOR_INVERTED,
            gearing = Constants.TransferConstants.GEARING
        )

        RobotType.Mode.SIMULATION -> TransferIOSim(
            gearing = Constants.TransferConstants.GEARING,
            moi = Constants.TransferConstants.MOI
        )

        RobotType.Mode.REPLAY -> object : TransferIO {}
    }
    
    public val inputs: TransferInputs = TransferInputs()

    init{
        io.setBrakeMode(true)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("Transfer", inputs)
    }

    fun getRollCommand(): Command {
        return runOnce {
            io.setBrakeMode(false)
            io.setVoltage(Constants.TransferConstants.TRANSFER_VOLTAGE)
        }
    }

    fun getStopCommand(): Command {
        return runOnce {
            io.setVoltage(Volt.zero())
            io.setBrakeMode(true)
        }
    }

    fun isDetectingCoral(): Trigger {
        return Trigger { inputs.coralDistance < Constants.TransferConstants.DETECTION_DISTANCE_THRESHOLD }
    }
}