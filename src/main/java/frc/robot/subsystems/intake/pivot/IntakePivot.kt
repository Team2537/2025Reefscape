package frc.robot.subsystems.intake.pivot

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotInputs
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands

class IntakePivot : SubsystemBase() {
    private val io: IntakePivotIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> IntakePivotIOKraken(
            Constants.IntakeConstants.PivotConstants.MOTOR_ID,
            Constants.IntakeConstants.PivotConstants.GEARING,
            Constants.IntakeConstants.PivotConstants.IS_MOTOR_INVERTED,
            Constants.IntakeConstants.PivotConstants.PID_GAINS,
            Constants.IntakeConstants.PivotConstants.FF_GAINS,
            Constants.IntakeConstants.PivotConstants.KG,
            Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_ACCELERATION,
            Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_CRUISE_VELOCITY
        )

        RobotType.Mode.SIMULATION -> IntakePivotIOSim(
            DCMotor.getKrakenX60Foc(1),
            Constants.IntakeConstants.PivotConstants.GEARING,
            Constants.IntakeConstants.PivotConstants.MOI,
            Constants.IntakeConstants.PivotConstants.PID_GAINS,
            Constants.IntakeConstants.PivotConstants.FF_GAINS,
            Constants.IntakeConstants.PivotConstants.KG
        )

        RobotType.Mode.REPLAY -> object : IntakePivotIO {}
    }

    private val inputs: IntakePivotInputs = IntakePivotInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IntakePivot", inputs)

        // TODO: add logging for outputs
    }

    // move intake arm into the down position, from the constants file
    fun getSlamDownCommand(): Command {
        return Commands.runOnce({
            io.setTargetAngle(Constants.IntakeConstants.PivotConstants.DOWN_ANGLE)
        })
    }

    // move intake arm into the up position, from the constants file
    fun getSlamUpCommand(): Command {
        return Commands.runOnce({
            io.setTargetAngle(Constants.IntakeConstants.PivotConstants.UP_ANGLE)
        })
    }
}