package frc.robot

import com.reduxrobotics.canand.CanandEventLoop
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.Autos
import frc.robot.commands.swerve.AlignmentCommand
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.swerve.Drivebase
import frc.robot.subsystems.vision.Vision
import lib.commands.not
import lib.math.geometry.FieldConstants
import lib.math.units.degrees
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.math.pow

object Robot : LoggedRobot() {
    val updateRateSec = 0.02

    val driverController = CommandXboxController(0)
    val operatorController = CommandXboxController(1)

    val godController: CommandXboxController = CommandXboxController(5)

    val drivebase: Drivebase
    val vision: Vision
    val superstructure: Superstructure
    val climb = Climb()

    val autos: Autos


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
        Logger.recordOutput("Git Dirty", if(DIRTY == 1) "DIRTY" else "CLEAN")
        Logger.recordOutput("Git Branch", GIT_BRANCH)
        Logger.recordOutput("Git SHA", GIT_SHA)
        Logger.recordOutput("Git Date", GIT_DATE)

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

        CanandEventLoop.getInstance()
        FieldConstants

        CommandScheduler.getInstance()
            .onCommandInitialize { command -> Logger.recordOutput("commands/${command.name}", true) }
        CommandScheduler.getInstance()
            .onCommandFinish { command -> Logger.recordOutput("commands/${command.name}", false) }

        drivebase = Drivebase()
        vision = Vision(drivebase::addVisionMeasurement)
        superstructure = Superstructure()

        autos = Autos(drivebase, superstructure, climb)


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
            driverController.rightStick(),
            3
        )

        driverController.leftTrigger().toggleOnTrue(
            AlignmentCommand.nodeAlignment(
                drivebase,
                FieldConstants.Reef.Side.LEFT,
                superstructure.coralPositionSupplier,
                driverController.y()
            )
        )

        operatorController.leftStick().onTrue(climb.getSendToPositionCommand { 60.0.degrees })

        driverController.rightTrigger().toggleOnTrue(
            AlignmentCommand.nodeAlignment(
                drivebase,
                FieldConstants.Reef.Side.RIGHT,
                superstructure.coralPositionSupplier,
                driverController.y()
            )
        )

        driverController.rightBumper().onTrue(AlignmentCommand.algaeAlignment(drivebase))

        for (i in 0..360 step 45) {
            driverController.pov(i).onTrue(AlignmentCommand.sourceAlignment(drivebase))
        }

        operatorController.povDown().onTrue(superstructure.getPrepL1Command())
        operatorController.povUp().onTrue(superstructure.getPrepL2Command())
        operatorController.a().onTrue(superstructure.getPrepL3Command())
        operatorController.y().onTrue(superstructure.getPrepL4Command())

        operatorController.rightTrigger().onTrue(superstructure.getPrepL3DealgaefyCmd())

        operatorController.x().onTrue(superstructure.getStowCommand())

        operatorController.rightBumper().and(
            operatorController.leftBumper()
        ).onTrue(superstructure.getSourceIntakeCommand())

        climb.defaultCommand = climb.getVoltageControlCommand { -operatorController.rightY * 12.0 }

        operatorController.rightStick().onTrue(superstructure.getClimbCommand())

        driverController.x().onTrue(superstructure.getScoreCommand())

        driverController.a().onTrue(superstructure.getDealgaefyCommand())
        driverController.b().onTrue(superstructure.getDealgaefyCommand())

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        superstructure.periodic()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
        autos.selectedRoutine.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()
        superstructure.getStowCommand().schedule()
    }

    override fun teleopPeriodic() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {
        MechanismVisualizer.updatePoses()
    }
}
