package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotType.Type.*
import frc.robot.RobotType
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.module.SwerveModule

class Drivebase : SubsystemBase("drivebase") {
    
    /**
     * The translations of the modules relative to the center of the robot.
     *
     * This array contains the translations of the modules relative to the center of the robot. The
     * order of the modules is as follows:
     * 0: Front Left
     * 1: Front Right
     * 2: Back Left
     * 3: Back Right
     *
     * The translations are in meters.
     */
    val moduleTranslations: Array<Translation2d> = when(RobotType.type){
        SWERVE_TEST_BOT -> arrayOf(
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0)
        )
        else -> arrayOf(
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0),
            Translation2d(0.0, 0.0)
        )
    }
    
    /**
     * The swerve modules on the robot.
     *
     * This array contains the swerve modules on the robot. The order of the modules is as follows:
     * 0: Front Left
     * 1: Front Right
     * 2: Back Left
     * 3: Back Right
     */
    val modules: Array<SwerveModule> = arrayOf(
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[0]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[1]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[2]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[3])
    )
    
    val gyro: GyroIO = when(RobotType.type){
        else -> TODO()
    }
}