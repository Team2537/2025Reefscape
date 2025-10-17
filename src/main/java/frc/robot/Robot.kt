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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.Autos
import frc.robot.commands.swerve.AlignmentCommand
import frc.robot.commands.swerve.WheelRadiusCharacterization
// import frc.robot.subsystems.intake.Intake
// import frc.robot.subsystems.transfer.Transfer
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructureGoals
// import frc.robot.subsystems.superstructure.SuperstructureGoals.L4
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
    // val intake: Intake
    // val transfer: Transfer

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

        autos = Autos(drivebase, superstructure)


        configureBindings()

        DriverStation.silenceJoystickConnectionWarning(true)

//        driverController.a().whileTrue(drivebase.driveSysId())
    }

    fun configureBindings() {
        val forwardAxis: java.util.function.DoubleSupplier = java.util.function.DoubleSupplier { -(MathUtil.applyDeadband(driverController.leftY, 0.05)) }
        val strafeAxis: java.util.function.DoubleSupplier = java.util.function.DoubleSupplier { -(MathUtil.applyDeadband(driverController.leftX, 0.05)) }
        val rotationAxis: java.util.function.DoubleSupplier = java.util.function.DoubleSupplier { -(MathUtil.applyDeadband(driverController.rightX, 0.05)) }
        val fieldOriented = java.util.function.BooleanSupplier { !driverController.leftStick().asBoolean }
        val boost = java.util.function.BooleanSupplier { driverController.leftBumper().asBoolean }
        val slow = java.util.function.BooleanSupplier { driverController.rightBumper().asBoolean }
        val headingSupplier: java.util.function.Supplier<Rotation2d?> = java.util.function.Supplier {
            if (driverController.povLeft().asBoolean) {
                Rotation2d.fromDegrees(-55.0)
            } else if (driverController.povRight().asBoolean) {
                Rotation2d.fromDegrees(55.0)
            } else {
                null
            }
        }

        drivebase.defaultCommand = drivebase.getDriveCmd(
            forwardAxis,
            strafeAxis,
            rotationAxis,
            fieldOriented,
            boost,
            slow,
            headingSupplier,
            3
        )

        // dealgaefy l2
        driverController.leftTrigger().onTrue(Commands.sequence(
            superstructure.getForceStateCommand { SuperstructureGoals.ALGAE_L2 },
            superstructure.getDealgaefyCommand(!driverController.leftTrigger())
            )
        )
        // dealgaefy l3
        driverController.rightTrigger().onTrue(Commands.sequence(
            superstructure.getForceStateCommand { SuperstructureGoals.ALGAE_L3 },
            superstructure.getDealgaefyCommand(!driverController.rightTrigger())
            )
        )
        
        // stow
        driverController.b().onTrue(superstructure.getSendToStateCommand { SuperstructureGoals.STOW })
        // score l1
        driverController.y().onTrue(
            Commands.sequence(
                superstructure.getForceStateCommand { SuperstructureGoals.L1 },
                superstructure.getScoreCommand(driverController.y())
            )
        )

        // auto align
        driverController.x().onTrue(AlignmentCommand.tagRelativeAlign(drivebase, vision, 0.45, 0.0).withTimeout(3.0))
        // intake
        driverController.a().onTrue(superstructure.getIntakeCommand(driverController.a()))

        // sysid
        driverController.povUp().onTrue(superstructure.arm.getDynamicSysID(SysIdRoutine.Direction.kForward))
        driverController.povDown().onTrue(superstructure.arm.getDynamicSysID(SysIdRoutine.Direction.kReverse))

        // driverController.rightTrigger().onTrue(Commands.sequence(superstructure.getForceStateCommand { SuperstructureGoals.L3 }, superstructure.getDealgaefyCommand()))
        // driverController.rightTrigger().onTrue(
        //     Commands.sequence(
        //         superstructure.getDealgaefyCommand().onlyIf(!superstructure.manipulator.isDetectingGamePiece()), // assume game piece is algae
        //         superstructure.getReleaseAlgaeCommand().onlyIf(superstructure.manipulator.isDetectingGamePiece()),
        //     )
        // )
        
        // FieldConstants.Reef.ReefFace.entries.forEach { face ->
        //     FieldConstants.Reef.Side.entries.forEach { side ->
        //         operatorController.getReefButton(face, side).whileTrue(
        //             AlignmentCommand.buttonBoardAlign(
        //                 drivebase,
        //                 face,
        //                 superstructure.isL4,
        //                 leftSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.LEFT),
        //                 rightSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.RIGHT),
        //                 centerSupplier = operatorController.getReefButton(face, FieldConstants.Reef.Side.CENTER)
        //             )
        //         )
        //     }
        // }

        // operatorController.getL1Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L1 })
        // operatorController.getL2Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L2 })
        // operatorController.getL3Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L3 })
        // operatorController.getL4Button().onTrue(superstructure.getForceStateCommand { SuperstructureGoals.L4 })


        // full intake command
        // driverController.b().onTrue(
        //     Commands.sequence(
        //         Commands.parallel(
        //             superstructure.getSendToStateCommand{SuperstructureGoals.STOW},
        //             intake.getDeployIntakeCommand(),
        //         ),
        //         Commands.waitUntil { superstructure.getStateAchievedTrigger(SuperstructureGoals.STOW).asBoolean },
        //         Commands.parallel(
        //             intake.getSpinRollersCommand(),
        //             transfer.getRollCommand(),
        //         ),
        //         Commands.waitUntil(transfer.isDetectingCoral()),
        //         intake.getStopRollersCommand(),
        //         intake.getRetractIntakeCommand(),
        //         superstructure.manipulator.getSpinRollersInSlowCommand(),
        //         Commands.deadline( // this deadline is for sim to avoid infinitely waiting
        //             Commands.waitSeconds(5.0),
        //             Commands.waitUntil(superstructure.manipulator.isDetectingGamePiece()),
        //         ),
        //         Commands.waitSeconds(0.1),
        //         transfer.getStopCommand(),
        //         superstructure.manipulator.getStopRollersCommand(),
        //     )
        // )

        // driverController.rightBumper().onTrue(
        //     superstructure.getScoreCommand(!driverController.rightBumper())
        // )

        // driverController.leftBumper().onTrue(
        //     superstructure.getSendToStateCommand { SuperstructureGoals.STOW }
        // )
    }

    override fun robotPeriodic() {
        // AdvantageScope setup
        // Logger.recordOutput("RobotPose", Pose2d(0.0, 0.0, Rotation2d(0.0)))
        // Logger.recordOutput("ZeroedComponentPoses", Pose3d())

        // Logger.recordOutput("FinalComponentPoses", Pose3d(
        //     0.28, 0.0, 0.275,
        //     Rotation3d(0.0, Math.sin(Timer.getTimestamp()), 0.0)
        //     )
        // )

        // val elevatorHeight: Double = superstructure.elevator.inputs.carriageHeight into Meters
        // val pivotAngle: Double = superstructure.manipulator.inputs.pivotAngularPosition into Radians
        // val intakeAngle: Double = intake.inputs.pivotLeftPosition into Radians

        val armAngle: Double = superstructure.arm.inputs.angle into Radians

        Logger.recordOutput(
            "FinalComponentPoses",
            // arm
            Pose3d(
                0.28, 0.0, 0.275,
                Rotation3d(0.0, armAngle - 1.0, 0.0)
            ),
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
