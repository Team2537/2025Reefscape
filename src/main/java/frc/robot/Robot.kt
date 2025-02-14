package frc.robot

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.util.datalog.DataLog
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.Autos
import frc.robot.commands.swerve.WheelRadiusCharacterization
import frc.robot.subsystems.Superstructure
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.swerve.Drivebase
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionIO
import lib.commands.not
import lib.math.geometry.FieldConstants
import lib.math.units.degrees
import lib.math.units.into
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

object Robot : LoggedRobot() {
    val updateRateMs = 0.02
    
    val driverController = CommandXboxController(0)
    val operatorController = CommandXboxController(1)

    val godController: CommandXboxController = CommandXboxController(5)
    
    val drivebase: Drivebase = Drivebase()
    val vision: Vision = Vision(drivebase::addVisionMeasurement)
    val superstructure = Superstructure()
    
    val autos = Autos(drivebase)
    
    
    
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
        
        when (RobotType.mode) {
            RobotType.Mode.REAL -> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
                
                PowerDistribution(1, PowerDistribution.ModuleType.kRev)
            }
            
            RobotType.Mode.SIMULATION -> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
            }
            
            RobotType.Mode.REPLAY -> {
                setUseTiming(false)
                
                val logFile = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logFile))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")))
            }
        }
        
        Logger.start()
        FieldConstants
        configureBindings()
        
        DriverStation.silenceJoystickConnectionWarning(true)

//        driverController.a().whileTrue(drivebase.driveSysId())
    }
    
    fun configureBindings() {
        drivebase.defaultCommand = drivebase.getDriveCmd(
            { -(MathUtil.applyDeadband(driverController.leftY, 0.05)) },
            { -(MathUtil.applyDeadband(driverController.leftX, 0.05)) },
            { -(MathUtil.applyDeadband(driverController.rightX, 0.05)) },
            !driverController.leftBumper(),
            { 1.0 },
            3
        )
        
        driverController.rightBumper().onTrue(drivebase.resetHeading())

//        operatorController.b().onTrue(elevator.getSendToHomeCmd())

        operatorController.povRight().onTrue(superstructure.getScoreNodeCmd(
            operatorController.leftTrigger().and(operatorController.rightTrigger()),
            driverController.leftTrigger(),
            Superstructure.SuperstructureSetpoint.L1
        ))

        operatorController.povDown().onTrue(superstructure.getScoreNodeCmd(
            operatorController.leftTrigger().and(operatorController.rightTrigger()),
            driverController.leftTrigger(),
            Superstructure.SuperstructureSetpoint.L2
        ))

        operatorController.povLeft().onTrue(superstructure.getScoreNodeCmd(
            operatorController.leftTrigger().and(operatorController.rightTrigger()),
            driverController.leftTrigger(),
            Superstructure.SuperstructureSetpoint.L3
        ))

        operatorController.povUp().onTrue(superstructure.getScoreNodeCmd(
            operatorController.leftTrigger().and(operatorController.rightTrigger()),
            driverController.leftTrigger(),
            Superstructure.SuperstructureSetpoint.L4
        ))
    }
    
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        superstructure.periodic()
        MechanismVisualizer.updatePoses()
    }
    
    override fun disabledInit() {}
    
    override fun disabledPeriodic() {}
    
    override fun autonomousInit() {
        autos.selectedRoutine.schedule()
    }
    
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
