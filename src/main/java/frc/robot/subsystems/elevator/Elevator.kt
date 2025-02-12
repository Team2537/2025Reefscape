package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.units.inches
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
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
    
    private var currentNodeHeight: FieldConstants.Reef.Level? = null
    
    fun getSendToNodeCmd(node: FieldConstants.Reef.Level): Command {
        currentNodeHeight = node
        return Commands.print("Sending to node $node")
            .andThen(runOnce { io.setElevatorHeightTarget(node.height) })
    }
    
    fun getMoveUpCmd(): Command {
        val levels = FieldConstants.Reef.Level.entries.sortedBy { it.height } // Ensure levels are sorted by height.
        
        if (currentNodeHeight == null) {
            return getSendToNodeCmd(levels.first())
        }
        
        val nextLevel = levels.find { it.height > currentNodeHeight!!.height }
        return nextLevel?.let { getSendToNodeCmd(it) } ?: Commands.idle(this)
    }
    
    fun getMoveDownCmd(): Command {
        val levels = FieldConstants.Reef.Level.entries.sortedBy { it.height } // Ensure levels are sorted by height.
        
        if (currentNodeHeight == null || currentNodeHeight == levels.first()) {
            return getSendToHomeCmd()
        }
        
        val previousLevel = levels.findLast { it.height < currentNodeHeight!!.height }
        return previousLevel?.let { getSendToNodeCmd(it) } ?: Commands.idle(this)
    }
    
    
    fun getSendToHomeCmd(): Command {
        currentNodeHeight = null
        return runOnce { io.setElevatorHeightTarget(0.0.inches) }
    }
    
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs(name, inputs)
        
        Logger.recordOutput("currentNodeHeight", currentNodeHeight.toString())
        
        MechanismVisualizer.setElevatorHeight(inputs.carriageHeight)
    }
    
    companion object {
        val maxHeight = 51.0 measuredIn Inches
    }
}