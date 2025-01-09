package frc.robot

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.swerve.Drivebase
import lib.commands.not
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

object Robot : LoggedRobot() {
    val driverController = CommandXboxController(0)

    val drivebase: Drivebase = Drivebase()

    init {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics.
        // Please retain this line so that Kotlin's growing use by teams is seen by FRC/WPI.
        HAL.report(
            tResourceType.kResourceType_Language,
            tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version
        )
        
        Logger.recordMetadata("Type", RobotType.type.toString())
        Logger.recordMetadata("Serial Number", HALUtil.getSerialNumber())
        
        when(RobotType.type){
            RobotType.Type.COMPETITION_BOT, RobotType.Type.SWERVE_TEST_BOT-> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
                
                PowerDistribution(1, PowerDistribution.ModuleType.kRev)
            }
            RobotType.Type.SIMULATION_BOT -> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
            }
            RobotType.Type.REPLAY -> {
                setUseTiming(false)
                
                val logFile = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logFile))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")))
            }
        }
        
        Logger.start()
        configureBindings()
    }

    fun configureBindings() {
        drivebase.defaultCommand = drivebase.getDriveCmd(
            { -Math.pow(driverController.leftY, 3.0) },
            { -Math.pow(driverController.leftX, 3.0) },
            { -Math.pow(driverController.rightX, 3.0) },
            !driverController.leftBumper(),
            { 1.0 }
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
