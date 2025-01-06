package frc.robot

import edu.wpi.first.hal.HALUtil
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger

object RobotType {
    enum class Type {
        SWERVE_TEST_BOT,
        COMPETITION_BOT,
        SIMULATION_BOT,
        REPLAY
    }
    
    val type = if(RobotBase.isReal()){
        when(HALUtil.getSerialNumber()){
            "COMPETITION_BOT" -> Type.COMPETITION_BOT
            "SWERVE_TEST_BOT" -> Type.SWERVE_TEST_BOT
            else -> Type.COMPETITION_BOT
        }
    } else {
        Type.SIMULATION_BOT
    }
    
    val isCompetitionBot = type == Type.COMPETITION_BOT
    val isSwerveTestBot = type == Type.SWERVE_TEST_BOT
    val isSimulationBot = type == Type.SIMULATION_BOT
    val isReplay = type == Type.REPLAY
}