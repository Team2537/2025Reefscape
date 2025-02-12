package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import frc.robot.subsystems.Superstructure
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.units.inches
import lib.math.units.measuredIn
import lib.math.units.volts
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import kotlin.math.max

class Elevator : SubsystemBase("elevator") {
    
    
    val io: ElevatorIO = when (RobotType.mode) {
        RobotType.Mode.SIMULATION -> ElevatorIOSim(
            gearing = 5.0,
            drumRadius = Inches.of(1.76),
            maxHeight = maxHeight,
            mass = 10.0 measuredIn Pounds,
            pidGains = PIDGains(kP = 10.0),
            ffGains = FeedforwardGains(kV = 1.77),
            kG = 0.19,
        )
        
        else -> object : ElevatorIO {}
    }
    
    val inputs: ElevatorIO.ElevatorInputs = ElevatorIO.ElevatorInputs()
    
    
    fun getSendToNodeCmd(setpoint: Superstructure.SuperstructureSetpoint): Command {
        return runOnce {
            io.setElevatorHeightTarget(setpoint.elevatorHeight)
        }
    }
    
    fun getManualMoveCommand(voltageSupplier: DoubleSupplier): Command {
        return run { io.setElevatorVoltage(voltageSupplier.asDouble.volts) }.handleInterrupt {
            io.setElevatorHeightTarget(
                inputs.carriageHeight
            )
        }
    }
    
    
    
    fun getSendToHomeCmd(): Command {
        return getSendToNodeCmd(Superstructure.SuperstructureSetpoint.HOME)
    }
    
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("superstructure/$name", inputs)
        
        MechanismVisualizer.setElevatorHeight(inputs.carriageHeight)
    }
    
    companion object {
        val maxHeight = 51.0 measuredIn Inches
    }
}