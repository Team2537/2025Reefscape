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
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.units.inches
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
import kotlin.math.max

class Elevator : SubsystemBase("elevator") {
    
    /**
     * The setpoints to send the elevator to when scoring on the reef
     * Differs from [FieldConstants.Reef.Level] as that is the actual reef height, this is the elevator height to score on the reef
     * @param height The height to send the elevator to
     */
    enum class Setpoint(val height: Distance) {
        HOME(0.0.inches),
        SOURCE(5.0.inches),
        L1(5.0.inches),
        L2(15.0.inches),
        L3(30.0.inches),
        L4(45.0.inches),
    }
    
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
    
    private var currentSetpoint = Setpoint.HOME

    val setpointIsHomeTrigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.HOME } }
    val setpointIsSourceTrigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.SOURCE } }
    val setpointIsL1Trigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.L1 } }
    val setpointIsL2Trigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.L2 } }
    val setpointIsL3Trigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.L3 } }
    val setpointIsL4Trigger: Trigger by lazy { Trigger { currentSetpoint == Setpoint.L4 } }
    
    fun getSendToNodeCmd(setpoint: Setpoint): Command {
        return runOnce {
            currentSetpoint = setpoint
            io.setElevatorHeightTarget(setpoint.height)
        }
    }
    
    fun getMoveUpCmd(): Command {
        return runOnce {
            currentSetpoint = Setpoint.entries.find { it.height > currentSetpoint.height } ?: currentSetpoint
            io.setElevatorHeightTarget(currentSetpoint.height)
        }
    }
    
    fun getMoveDownCmd(): Command {
        return runOnce {
            currentSetpoint = Setpoint.entries.findLast { it.height < currentSetpoint.height } ?: currentSetpoint
            io.setElevatorHeightTarget(currentSetpoint.height)
        }
    }
    
    
    fun getSendToHomeCmd(): Command {
        return getSendToNodeCmd(Setpoint.HOME)
    }
    
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs(name, inputs)
        
        Logger.recordOutput("$name/setpoint", currentSetpoint)
        
        MechanismVisualizer.setElevatorHeight(inputs.carriageHeight)
    }
    
    companion object {
        val maxHeight = 51.0 measuredIn Inches
    }
}