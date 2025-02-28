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
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.epsilonEquals
import lib.math.units.measuredIn
import lib.math.units.volts
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class Elevator : SubsystemBase("elevator") {
    
    
    val io: ElevatorIO = when (RobotType.mode) {
        RobotType.Mode.SIMULATION -> ElevatorIOSim(
            gearing = 5.0,
            drumRadius = Inches.of(1.76),
            maxHeight = Constants.ElevatorConstants.maxHeight,
            mass = 10.0 measuredIn Pounds,
            pidGains = PIDGains(kP = 10.0),
            ffGains = FeedforwardGains(kV = 1.77),
            kG = 0.19,
        )
        RobotType.Mode.REAL -> ElevatorIOKraken(
            gearRatio = 5.0,
            drumRadius = Inches.of(1.76),
            leftMotorId = 21,
            rightMotorId = 20,
            pidGains = PIDGains(kP = 10.0),
            ffGains = FeedforwardGains(kS = 0.082337, kV = 0.015551),
            kG = 0.44098,
            motionMagicCruiseVelocity = MetersPerSecond.of(2.5),
            motionMagicAcceleration = MetersPerSecondPerSecond.of(50.0),
            motionMagicJerk = 0.0
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
            val height = height.get()
            setpoint.mut_replace(height)
            io.setElevatorHeightTarget(
                height
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
        
        MechanismVisualizer.setElevatorHeight(inputs.carriageHeight)
    }
}