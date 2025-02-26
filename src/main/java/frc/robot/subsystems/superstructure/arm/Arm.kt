package frc.robot.subsystems.superstructure.arm

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import frc.robot.subsystems.superstructure.Superstructure
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.epsilonEquals
import lib.math.units.volts
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class Arm : SubsystemBase("arm") {
    val io: ArmIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> ArmIOKraken(
            motorID = Constants.ArmConstants.MOTOR_ID,
            inverted = Constants.ArmConstants.IS_MOTOR_INVERTED,
            gearing = Constants.ArmConstants.GEARING,
            pidGains = PIDGains(kP = 30.0),
            ffGains = FeedforwardGains(kS = 0.12782, kV = 4.97),
            kG = 0.45,
            velocityLimit = RotationsPerSecond.of(1.0),
            accelerationLimit = RotationsPerSecondPerSecond.of(1.0),
        )

        RobotType.Mode.SIMULATION -> ArmIOSim(
            motor = DCMotor.getKrakenX60Foc(1),
            gearing = 50.0,
            moi = KilogramSquareMeters.of(0.164586918),
            pidGains = PIDGains(kP = 100.0),
            ffGains = FeedforwardGains(kV = 0.99),
            kG = 0.21,
        )

        else -> object : ArmIO {}
    }

    val inputs: ArmIO.ArmInputs = ArmIO.ArmInputs()

    private val toleranceTriggerMap = mutableMapOf<Angle, Trigger>()

    val setpoint: MutAngle = inputs.motorRelativePosition.mutableCopy()

    val sysidRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            Volts.per(Second).of(1.0),
            Volts.of(4.0),
            Seconds.of(4.0),
            { state -> Logger.recordOutput("superstructure/$name/sysid", state.toString()) },
        ),
        SysIdRoutine.Mechanism(
            { voltage: Voltage -> io.setVoltage(voltage) },
            null,
            this
        )
    )

    fun getDynamicTest(direction: SysIdRoutine.Direction): Command {
        return sysidRoutine.dynamic(direction)
    }

    fun getQuasistaticTest(direction: Direction): Command {
        return sysidRoutine.quasistatic(direction)
    }

    fun getAngleInToleranceTrigger(tolerance: Angle): Trigger {
        return toleranceTriggerMap.getOrPut(tolerance) {
            Trigger {
                inputs.motorRelativePosition.epsilonEquals(setpoint, tolerance)
            }
        }
    }

    fun getSendToAngleCmd(angle: Supplier<Angle>): Command {
        return runOnce {
            val angle = angle.get()
            setpoint.mut_replace(angle)
            io.setAngle(angle)
        }
    }

    fun getManualMoveCmd(voltage: DoubleSupplier): Command {
        return run { io.setVoltage(voltage.asDouble.volts) }.handleInterrupt { io.setAngle(inputs.motorAbsolutePosition) }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("superstructure/$name", inputs)

        MechanismVisualizer.setArmAngle(inputs.motorRelativePosition)
    }
}