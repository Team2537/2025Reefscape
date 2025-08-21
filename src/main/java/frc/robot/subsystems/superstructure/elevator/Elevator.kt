package frc.robot.subsystems.superstructure.elevator

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
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
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import lib.math.units.epsilonEquals
import lib.math.units.measuredIn
import lib.math.units.volts
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class Elevator : SubsystemBase("elevator") {
    
    
    val io: ElevatorIO = when (RobotType.mode) {
        RobotType.Mode.SIMULATION -> ElevatorIOSim(
            gearing = Constants.ElevatorConstants.GEAR_RATIO,
            drumRadius = Constants.ElevatorConstants.DRUM_RADIUS,
            maxHeight = Constants.ElevatorConstants.maxHeight,
            mass = Constants.ElevatorConstants.SIM_MASS,
            pidGains = PIDGains(kP = Constants.ElevatorConstants.SIM_KP),
            ffGains = FeedforwardGains(kV = Constants.ElevatorConstants.SIM_KV),
            kG = Constants.ElevatorConstants.SIM_KG,
        )
        RobotType.Mode.REAL -> ElevatorIOKraken(
            gearRatio = Constants.ElevatorConstants.GEAR_RATIO,
            drumRadius = Constants.ElevatorConstants.DRUM_RADIUS,
            leftMotorId = Constants.ElevatorConstants.LEFT_MOTOR_ID,
            rightMotorId = Constants.ElevatorConstants.RIGHT_MOTOR_ID,
            pidGains = PIDGains(kP = Constants.ElevatorConstants.KP),
            ffGains = FeedforwardGains(kS = Constants.ElevatorConstants.KS, kV = Constants.ElevatorConstants.KV),
            kG = Constants.ElevatorConstants.KG,
            motionMagicCruiseVelocity = Constants.ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY,
            motionMagicAcceleration = Constants.ElevatorConstants.MOTION_MAGIC_ACCELERATION,
            motionMagicJerk = Constants.ElevatorConstants.MOTION_MAGIC_JERK
        )
        
        else -> object : ElevatorIO {}
    }
    
    val inputs: ElevatorIO.ElevatorInputs = ElevatorIO.ElevatorInputs()
    
    private val toleranceTriggerMap = mutableMapOf<Distance, Trigger>()
    
    private val setpoint = inputs.carriageHeight.mutableCopy()

    private val sysidRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            Seconds.of(5.0),
            { state -> Logger.recordOutput("superstructure/$name/sysid", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            {voltage: Voltage -> io.setElevatorVoltage(voltage) },
            null,
            this
        )
    )
    
    fun getPositionInToleranceTrigger(tolerance: Distance): Trigger {
        return toleranceTriggerMap.getOrPut(tolerance) {
            Trigger {
                inputs.carriageHeight.epsilonEquals(setpoint, tolerance)
            }
        }
    }
    
    fun getManualMoveCommand(voltageSupplier: DoubleSupplier): Command {
        return run { io.setElevatorVoltage(voltageSupplier.asDouble.volts) }.handleInterrupt {
            io.setElevatorHeightTarget(
                inputs.carriageHeight
            )
        }
    }
    
    fun getMoveToHeightCommand(height: Supplier<Distance>): Command {
        return runOnce {
            val targetHeight = height.get()
            setpoint.mut_replace(targetHeight)
            io.setElevatorHeightTarget(
                targetHeight
            )
        }
    }

    fun getDynamicSysID(direction: Direction): Command {
        val stopCriteria = if(direction == Direction.kForward) {
            { inputs.carriageHeight > Constants.ElevatorConstants.maxHeight - Inches.of(4.0) }
        } else {
            { inputs.carriageHeight < Inches.of(4.0) }
        }

        return sysidRoutine.dynamic(direction).until(stopCriteria)
    }

    fun getQuasistaticSysID(direction: Direction): Command {
        val stopCriteria = if(direction == Direction.kForward) {
            { inputs.carriageHeight > Constants.ElevatorConstants.maxHeight - Inches.of(4.0) }
        } else {
            { inputs.carriageHeight < Inches.of(4.0) }
        }

        return sysidRoutine.quasistatic(direction).until(stopCriteria)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("superstructure/$name", inputs)
    }

    override fun simulationPeriodic() {
        MechanismVisualizer.setElevatorHeight(inputs.carriageHeight)
    }
}