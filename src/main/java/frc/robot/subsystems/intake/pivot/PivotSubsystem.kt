package frc.robot.subsystems.intake.pivot

import com.sun.org.apache.xml.internal.security.Init.init
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.intake.pivot.IntakePivotIOKraken
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*

class PivotSubsystem: SubsystemBase() {
    private val io: IntakePivotIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> IntakePivotIOKraken(
            Constants.IntakeConstants.PivotConstants.MOTOR_ID,
            Constants.IntakeConstants.PivotConstants.GEARING,
            Constants.IntakeConstants.PivotConstants.IS_MOTOR_INVERTED,
            Constants.IntakeConstants.PivotConstants.kP,
            Constants.IntakeConstants.PivotConstants.kI,
            Constants.IntakeConstants.PivotConstants.kD,
            Constants.IntakeConstants.PivotConstants.kS,
            Constants.IntakeConstants.PivotConstants.kV,
            Constants.IntakeConstants.PivotConstants.kA,
            Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_ACCELERATION,
            Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_CRUISE_VELOCITY,
            Constants.IntakeConstants.PivotConstants.MOTION_MAGIC_JERK
        )

        Constants.RobotConstants.Mode.SIM -> IntakePivotIOSim(
                DCMotor.getKrakenX60(1),
                Constants.IntakeConstants.PivotConstants.GEARING,
                Constants.IntakeConstants.PivotConstants.MOI,
                Constants.IntakeConstants.PivotConstants.PID_GAINS,
                Constants.IntakeConstants.PivotConstants.FF_GAINS,
                Constants.IntakeConstants.PivotConstants.kG
        )

        Constants.RobotConstants.Mode.REPLAY -> object: IntakePivotIOKraken()

       init{
           io.setBrakeMode(true)
        }

    }