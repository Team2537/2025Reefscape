package frc.robot

import edu.wpi.first.hal.HALUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger

object RobotType {
    enum class Type {
        ROBOT_2025_COMP,
        ROBOT_2025_SWERVE_BASE
    }
    
    enum class Mode(val defaultRobotType: Type) {
        REAL(Type.ROBOT_2025_COMP),
        SIMULATION(Type.ROBOT_2025_COMP),
        REPLAY(Type.ROBOT_2025_COMP)
    }
    
    val isReplay: Boolean = false
    val mode: Mode = if(RobotBase.isReal()) Mode.REAL else if(isReplay) Mode.REPLAY else Mode.SIMULATION
    val type: Type = mode.defaultRobotType
    
    val isTuning: Boolean = !DriverStation.isFMSAttached()
}