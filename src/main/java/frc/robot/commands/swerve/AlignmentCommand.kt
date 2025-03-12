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
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.swerve.Drivebase
import lib.math.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.geometry.nudge
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
    val rotationPID: PIDGains,
    val endStateSupplier: Supplier<Drivebase.Constants.AlignmentState>
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
    private var endState: Drivebase.Constants.AlignmentState = Drivebase.Constants.AlignmentState.DRIVING

    init {
        addRequirements(drivebase)
    }

    override fun initialize() {
        pose = null
        drivebase.alignmentState = Drivebase.Constants.AlignmentState.ALIGNING
        endState = endStateSupplier.get()

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

    override fun end(interrupted: Boolean) {
        drivebase.alignmentState = endState
    }

    companion object {
        fun buttonBoardAlign(
            drivebase: Drivebase,
            side: FieldConstants.Reef.ReefFace,
            coralDistanceSupplier: Supplier<Distance>,
            isL4: Trigger,
            leftSupplier: Trigger,
            rightSupplier: Trigger,
            centerSupplier: Trigger,
        ): Command {
            return AlignmentCommand(
                drivebase,
                {
                    var targetPose = FieldConstants.Reef.floorAlignmentPoses[side.ordinal]

                    when {
                        leftSupplier.asBoolean
                                && !rightSupplier.asBoolean
                                && !centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                x = if(isL4.asBoolean) -backupL4.into(Meters) else 0.0,
                                y = (FieldConstants.Reef.sideOffset + coralDistanceSupplier.get()).into(Meters)
                            )
                        }

                        rightSupplier.asBoolean
                                && !leftSupplier.asBoolean
                                && !centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                x = if(isL4.asBoolean) backupL4.into(Meters) else 0.0,
                                y = ((-FieldConstants.Reef.sideOffset) + coralDistanceSupplier.get()).into(Meters)
                            )
                        }

                        (rightSupplier.asBoolean && leftSupplier.asBoolean)
                                || centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                x = 0.0,
                                y = (centerOffset).into(Meters)
                            )
                        }
                    }

                    targetPose.takeIf { it.translation.getDistance(drivebase.pose.translation) <= 1.5 }
                },
                PIDGains(kP = 5.0),
                PIDGains(kP = 5.0),
                {
                    when {
                        (rightSupplier.asBoolean || leftSupplier.asBoolean) && !centerSupplier.asBoolean ->
                            if(isL4.asBoolean) Drivebase.Constants.AlignmentState.ALIGNED_L4_CORAL else Drivebase.Constants.AlignmentState.ALIGNED_LOW_CORAL
                        centerSupplier.asBoolean -> Drivebase.Constants.AlignmentState.ALIGNED_ALGAE
                        else -> Drivebase.Constants.AlignmentState.DRIVING
                    }
                }
            )
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
                rotationPID = PIDGains(5.0, 0.0, 0.0),
                { Drivebase.Constants.AlignmentState.ALIGNED_SOURCE }
            ).withName("SourceAlignmentCommand").withTimeout(3.0)
        }

        val leftOffset: Distance = Inches.of(6.0)
        val rightOffset: Distance = Inches.of(3.5)
        val centerOffset: Distance = Inches.of(2.0)
        val backupL4: Distance = Inches.of(3.0)

        val alignLimits: PathConstraints = PathConstraints(
            MetersPerSecond.of(1.5),
            MetersPerSecondPerSecond.of(8.0),
            DegreesPerSecond.of(360 + 180.0),
            DegreesPerSecondPerSecond.of(720.0)
        )
    }


}