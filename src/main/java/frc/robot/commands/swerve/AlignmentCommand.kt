package frc.robot.commands.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.subsystems.swerve.Drivebase
import lib.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.units.inches
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.function.BooleanSupplier
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI

class AlignmentCommand(
    val drivebase: Drivebase,
    val poseSupplier: Supplier<Pose2d?>,
    val translationPID: PIDGains,
    val rotationPID: PIDGains
) : Command() {
    private val xPid = PIDController(
        translationPID.kP,
        translationPID.kI,
        translationPID.kD
    ).apply { setTolerance(Units.inchesToMeters(0.5)) }

    private val yPid = PIDController(
        translationPID.kP,
        translationPID.kI,
        translationPID.kD
    ).apply { setTolerance(Units.inchesToMeters(0.5)) }
    private val rotPid =
        PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD).apply {
            enableContinuousInput(0.0, 2 * PI)
            setTolerance(Units.degreesToRadians(2.0))
        }

    private var pose: Pose2d? = null

    init {
        addRequirements(drivebase)
    }

    override fun initialize() {
        pose = null

        xPid.reset()
        yPid.reset()
        rotPid.reset()

        pose = poseSupplier.get()
        Logger.recordOutput("commands/$name/Target Pose", Pose2d.struct, pose)
    }

    override fun execute() {
        pose?.let {
            val translation = drivebase.pose.translation
            val rotation = drivebase.pose.rotation

            val xOutput = xPid.calculate(translation.x, pose!!.translation.x)
            val yOutput = yPid.calculate(translation.y, pose!!.translation.y)
            val rotOutput = rotPid.calculate(rotation.radians, pose!!.rotation.radians)

            drivebase.applyChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, rotOutput, rotation),
                alignLimits
            )
        } ?: return
    }

    override fun isFinished(): Boolean {
        println("xPid: ${xPid.atSetpoint()} - yPid: ${yPid.atSetpoint()} - rotPid: ${rotPid.atSetpoint()} - pose: $pose")
        return (xPid.atSetpoint() && yPid.atSetpoint() && rotPid.atSetpoint()) || pose == null
    }

    companion object {
        fun nodeAlignment(
            drivebase: Drivebase,
            side: FieldConstants.Reef.Side,
            coralDistanceSupplier: Supplier<Distance>,
            isL4: BooleanSupplier
        ): Command {
            return AlignmentCommand(
                drivebase = drivebase,
                poseSupplier = {
                    val currPose = drivebase.pose.let {
                        if (AutoBuilder.shouldFlip()) it.flipped() else it
                    }

                    val closestReefSide = currPose.nearest(FieldConstants.Reef.floorAlignmentPoses)

                    val baseOffset = if (side == FieldConstants.Reef.Side.LEFT) 6.5.inches else 4.5.inches

                    val sideOffsetDistance =
                        (FieldConstants.Reef.sideOffset / (if (side == FieldConstants.Reef.Side.LEFT) 2.0 else -2.0)) + baseOffset + coralDistanceSupplier.get()

                    val targetPose = closestReefSide.transformBy(
                        Transform2d(
                            Translation2d(
                                if (isL4.asBoolean) Units.inchesToMeters(-3.5) else 0.0,
                                sideOffsetDistance into Meters
                            ),
                            Rotation2d()
                        )
                    ).let { if (AutoBuilder.shouldFlip()) it.flipped() else it }

                    val distanceToTarget = targetPose.translation.getDistance(drivebase.pose.translation)

                    targetPose.takeIf { distanceToTarget <= 1.5 }
                },
                translationPID = PIDGains(5.0, 0.0, 0.05),
                rotationPID = PIDGains(5.0, 0.0, 0.0)
            ).withName("NodeAlignmentCommand(${side.name})").withTimeout(3.0)
        }

        fun algaeAlignment(
            drivebase: Drivebase
        ): Command {
            return AlignmentCommand(
                drivebase,
                {
                    val currPose = drivebase.pose.let { if (AutoBuilder.shouldFlip()) it.flipped() else it }
                    val targetPose = currPose.nearest(FieldConstants.Reef.floorAlignmentPoses)
                        .let { if (AutoBuilder.shouldFlip()) it.flipped() else it }

                    targetPose.takeIf { it.translation.getDistance(drivebase.pose.translation) <= 1.5 }
                },
                translationPID = PIDGains(5.0, 0.0, 0.05),
                rotationPID = PIDGains(5.0, 0.0, 0.0)
            ).withName("AlgaeAlignmentCommand").withTimeout(3.0)
        }

        fun sourceAlignment(
            drivebase: Drivebase,
        ): Command {
            return AlignmentCommand(
                drivebase,
                {
                    val currPose = drivebase.pose.let { if (AutoBuilder.shouldFlip()) it.flipped() else it }
                    val targetPose = currPose.nearest(
                        listOf(
                            FieldConstants.Source.blueTopSourceCenter,
                            FieldConstants.Source.blueBottomSourceCenter
                        )
                    ).let { if (AutoBuilder.shouldFlip()) it.flipped() else it }

                    println("targetPose: $targetPose")
                    targetPose.takeIf { it.translation.getDistance(drivebase.pose.translation) <= 1.0 }
                },
                translationPID = PIDGains(5.0, 0.0, 0.05),
                rotationPID = PIDGains(5.0, 0.0, 0.0)
            ).withName("SourceAlignmentCommand").withTimeout(3.0)
        }
    }

    val alignLimits: PathConstraints = PathConstraints(
        MetersPerSecond.of(1.5),
        MetersPerSecondPerSecond.of(8.0),
        DegreesPerSecond.of(360 + 180.0),
        DegreesPerSecondPerSecond.of(720.0)
    )
}