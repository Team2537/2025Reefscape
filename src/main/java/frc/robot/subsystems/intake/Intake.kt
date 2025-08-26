package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
// import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotType
import edu.wpi.first.wpilibj2.command.Commands
import lib.math.controllers.gains.PIDGains
import lib.math.controllers.gains.FeedforwardGains

class Intake : SubsystemBase() {
    private val io: IntakeIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> IntakeIOKraken(
            leftMotorID = Constants.IntakeConstants.LEFT_PIVOT_MOTOR_ID,
            rightMotorID = Constants.IntakeConstants.RIGHT_PIVOT_MOTOR_ID,
            canandcolorID = Constants.IntakeConstants.CANANDCOLOR_ID,
            rollerMotorID = Constants.IntakeConstants.ROLLER_MOTOR_ID,
            leftInverted = Constants.IntakeConstants.LEFT_PIVOT_INVERTED,
            rightInverted = Constants.IntakeConstants.RIGHT_PIVOT_INVERTED,
            rollerInverted = Constants.IntakeConstants.ROLLER_INVERTED,
            gearing = Constants.IntakeConstants.PIVOT_GEARING,
            pidGains = PIDGains(kP = Constants.IntakeConstants.REAL_PIVOT_KP),
            ffGains = FeedforwardGains(kV = Constants.IntakeConstants.REAL_PIVOT_KV, kA = Constants.IntakeConstants.REAL_PIVOT_KA),
            kG = Constants.IntakeConstants.REAL_PIVOT_KG,
            velocityLimit = Constants.IntakeConstants.REAL_PIVOT_VELOCITY_LIMIT,
            accelerationLimit = Constants.IntakeConstants.REAL_PIVOT_ACCELERATION_LIMIT,
            jerkLimit = Constants.IntakeConstants.REAL_PIVOT_JERK_LIMIT
        )

        RobotType.Mode.SIMULATION -> IntakeIOSim(
            gearing = Constants.IntakeConstants.PIVOT_GEARING,
            moi = Constants.IntakeConstants.SIM_MOI,
            pidGains = PIDGains(kP = Constants.IntakeConstants.SIM_PIVOT_KP),
            ffGains = FeedforwardGains(kV = Constants.IntakeConstants.SIM_PIVOT_KV, kA = Constants.IntakeConstants.SIM_PIVOT_KA),
            kG = Constants.IntakeConstants.SIM_PIVOT_KG,
        )

        RobotType.Mode.REPLAY -> object : IntakeIO {}
    }
    
    val inputs: IntakeInputs = IntakeInputs()

    // for pivot only
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
    val sysidRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null,
            { state -> Logger.recordOutput("intake/roller/sysid", state.toString()) },
        ),
        SysIdRoutine.Mechanism(
            { voltage: Voltage -> io.setPivotVoltage(voltage) },
            null,
            this
        )
    )

    init{
        io.setPivotBrakeMode(true)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("Intake", inputs)

    }

    fun getDeployIntakeCommand(): Command {
        return runOnce {
            io.setPivotAngle(Constants.IntakeConstants.PIVOT_DEPLOYED_ANGLE)
        }
    }

    fun getRetractIntakeCommand(): Command {
        return runOnce {
            io.setPivotAngle(Constants.IntakeConstants.PIVOT_RETRACTED_ANGLE)
        }
    }

    fun getSpinRollersCommand(): Command {
        return runOnce {
            io.setRollerVoltage(Constants.IntakeConstants.ROLLER_VOLTAGE)
        }
    }

    fun getDynamicTest(direction: SysIdRoutine.Direction): Command {
        return sysidRoutine.dynamic(direction)
    }

    fun getQuasistaticTest(direction: Direction): Command {
        return sysidRoutine.quasistatic(direction)
    }
}