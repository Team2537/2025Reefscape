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
import lib.math.controllers.gains.PIDGains
import lib.math.controllers.gains.FeedforwardGains
import frc.robot.subsystems.superstructure.manipulator.*

class Manipulator : SubsystemBase() {
    public val io: ManipulatorIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> ManipulatorIOKraken(
            pivotMotorID = Constants.ManipulatorConstants.PIVOT_MOTOR_ID,
            rollerMotorID = Constants.ManipulatorConstants.ROLLER_MOTOR_ID,
            canandcolorID = Constants.ManipulatorConstants.CANANDCOLOR_ID,
            pivotInverted = Constants.ManipulatorConstants.PIVOT_INVERTED,
            rollerInverted = Constants.ManipulatorConstants.ROLLER_INVERTED,
            pivotGearing = Constants.ManipulatorConstants.PIVOT_GEARING,
            rollerGearing = Constants.ManipulatorConstants.ROLLER_GEARING,
            pivotPIDGains = PIDGains(kP = Constants.ManipulatorConstants.REAL_PIVOT_KP),
            pivotFFGains = FeedforwardGains(kV = Constants.ManipulatorConstants.REAL_PIVOT_KV, kA = Constants.ManipulatorConstants.REAL_PIVOT_KA),
            pivotKG = Constants.ManipulatorConstants.REAL_PIVOT_KG,
            pivotVelocityLimit = Constants.ManipulatorConstants.REAL_PIVOT_VELOCITY_LIMIT,
            pivotAccelerationLimit = Constants.ManipulatorConstants.REAL_PIVOT_ACCELERATION_LIMIT,
            pivotJerkLimit = Constants.ManipulatorConstants.REAL_PIVOT_JERK_LIMIT
        )

        RobotType.Mode.SIMULATION -> ManipulatorIOSim(
            pivotGearing = Constants.ManipulatorConstants.PIVOT_GEARING,
            rollerGearing = Constants.ManipulatorConstants.ROLLER_GEARING,
            pivotMoi = Constants.ManipulatorConstants.SIM_PIVOT_MOI,
            rollerMoi = Constants.ManipulatorConstants.SIM_ROLLER_MOI,
            pivotPIDGains = PIDGains(kP = Constants.ManipulatorConstants.SIM_PIVOT_KP),
            pivotFFGains = FeedforwardGains(kV = Constants.ManipulatorConstants.SIM_PIVOT_KV, kA = Constants.ManipulatorConstants.SIM_PIVOT_KA),
            pivotKG = Constants.ManipulatorConstants.SIM_PIVOT_KG,
        )

        RobotType.Mode.REPLAY -> object : ManipulatorIO {}
    }
    
    public val inputs: ManipulatorInputs = ManipulatorInputs()

    // for pivot only
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
    val sysidRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null,
            { state -> Logger.recordOutput("manipulator/pivot/sysid", state.toString()) },
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

        Logger.processInputs("Manipulator", inputs)

    }

    fun getSendToAngleCommand(angle: Angle): Command {
        return runOnce {
            io.setPivotTargetAngle(angle)
        }
    }

    fun getSpinRollersInCommand(): Command {
        return runOnce {
            io.setRollerVoltage(Constants.ManipulatorConstants.ROLLER_IN_VOLTAGE)
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

    fun getDynamicTest(direction: SysIdRoutine.Direction): Command {
        return sysidRoutine.dynamic(direction)
    }

    fun getQuasistaticTest(direction: Direction): Command {
        return sysidRoutine.quasistatic(direction)
    }
}
