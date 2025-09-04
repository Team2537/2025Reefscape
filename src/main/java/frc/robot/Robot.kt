package frc.robot

import com.reduxrobotics.canand.CanandEventLoop
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.swerve.AlignmentCommand
import frc.robot.commands.swerve.WheelRadiusCharacterization
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.transfer.Transfer
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructureGoals
import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
import frc.robot.subsystems.swerve.Drivebase
import frc.robot.subsystems.vision.Vision
import lib.commands.not
import lib.controllers.CommandButtonBoard
import lib.math.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.geometry.nudge
import lib.math.units.degrees
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.util.*
import kotlin.math.pow
import edu.wpi.first.wpilibj.Timer;
import lib.math.units.into

object Robot : LoggedRobot() {
    val updateRateSec = 0.02

    val driverController = CommandXboxController(0)
    val operatorController = CommandButtonBoard(1, 2)

    val godController: CommandXboxController = CommandXboxController(5)

    val drivebase: Drivebase
    val vision: Vision
    val superstructure: Superstructure
    val intake: Intake
    val transfer: Transfer

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
        Logger.recordOutput("Git Dirty", if (DIRTY == 1) "DIRTY" else "CLEAN")
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

        CameraServer.startAutomaticCapture()

        drivebase = Drivebase()
        vision = Vision(drivebase::addVisionMeasurement)
        superstructure = Superstructure()
        intake = Intake()
        transfer = Transfer()

        autos = Autos(drivebase, superstructure)


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
            driverController.leftTrigger(),
            {
                if (driverController.povLeft().asBoolean)
                    Rotation2d.fromDegrees(-55.0)
                else if (driverController.povRight().asBoolean) Rotation2d.fromDegrees(55.0)
                else null
            },
            3
        )

        FieldConstants.Reef.ReefFace.entries.forEach { face ->
            FieldConstants.Reef.Side.entries.forEach { side ->
                operatorController.getReefButton(face, side).whileTrue(
                    AlignmentCommand.buttonBoardAlign(
                        drivebase,
                        face,
                        superstructure.isL4,
                        leftSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.LEFT),
                        rightSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.RIGHT),
                        centerSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.CENTER)
                    )
                )
            }
        }

        operatorController.getL1Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L1 })
        operatorController.getL2Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L2 })
        operatorController.getL3Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L3 })
        operatorController.getL4Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L4 })

        operatorController.getStowButton().onTrue(superstructure.getSendToStateCommand { SuperstructureGoals.STOW })
        operatorController.getActionButton().onTrue(superstructure.getScoreCommand(operatorController.getActionButton()))

        driverController.x().onTrue(AlignmentCommand.tagRelativeAlign(drivebase, vision, 0.45, 0.0))


        // driverController.rightTrigger().onTrue(Commands.sequence(superstructure.getForceStateCommand { SuperstructureGoals.L3 }, superstructure.getDealgaefyCommand()))
        driverController.rightTrigger().onTrue(
            Commands.sequence(
                superstructure.getDealgaefyCommand().onlyIf(!superstructure.manipulator.isDetectingGamePiece()), // assume game piece is algae
                superstructure.getReleaseAlgaeCommand().onlyIf(superstructure.manipulator.isDetectingGamePiece()),
            )
        )
        
        // deploy/retract intake
        driverController.a().onTrue(intake.toggleIntakeCommand())

        // full intake command
        driverController.b().onTrue(
            Commands.sequence(
                Commands.parallel(
                    superstructure.getSendToStateCommand{SuperstructureGoals.STOW},
                    intake.getDeployIntakeCommand(),
                ),
                Commands.waitUntil { superstructure.getStateAchievedTrigger(SuperstructureGoals.STOW).asBoolean },
                Commands.parallel(
                    intake.getSpinRollersCommand(),
                    transfer.getRollCommand(),
                ),
                Commands.waitUntil(transfer.isDetectingCoral()),
                intake.getStopRollersCommand(),
                intake.getRetractIntakeCommand(),
                superstructure.manipulator.getSpinRollersInSlowCommand(),
                Commands.deadline( // this deadline is for sim to avoid infinitely waiting
                    Commands.waitSeconds(5.0),
                    Commands.waitUntil(superstructure.manipulator.isDetectingGamePiece()),
                ),
                Commands.waitSeconds(0.1),
                transfer.getStopCommand(),
                superstructure.manipulator.getStopRollersCommand(),
            )
        )

        // driverController.rightBumper().onTrue(
        //     superstructure.getScoreCommand(!driverController.rightBumper())
        // )

        // driverController.leftBumper().onTrue(
        //     superstructure.getSendToStateCommand { SuperstructureGoals.STOW }
        // )
    }

    override fun robotPeriodic() {
        // AdvantageScope setup
        // Logger.recordOutput("RobotPose", Pose2d(3.0, 2.0, Rotation2d(0.0)))
        // Logger.recordOutput("ZeroedComponentPoses", Pose3d(), Pose3d(), Pose3d())

        val elevatorHeight: Double = superstructure.elevator.inputs.carriageHeight into Meters
        val pivotAngle: Double = superstructure.manipulator.inputs.pivotAngularPosition into Radians
        val intakeAngle: Double = intake.inputs.pivotLeftPosition into Radians

        Logger.recordOutput(
            "FinalComponentPoses",
            // intake
            Pose3d(
                0.33655, 0.0, 0.24765,
                Rotation3d(0.0, intakeAngle - 1.5, 0.0)
            ),
            // wrist
            Pose3d(
                -0.325374, 0.0, 0.2437638+elevatorHeight,
                Rotation3d(0.0, pivotAngle-0.7, 0.0)
            ),
            // no wrist
            Pose3d(
                -0.325374, 0.0, 0.2437638+elevatorHeight,
                Rotation3d(0.0, 0.0, 0.0)
            )
        )


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
        superstructure.getSendToStateCommand({ SuperstructureGoals.STOW }).schedule()
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
