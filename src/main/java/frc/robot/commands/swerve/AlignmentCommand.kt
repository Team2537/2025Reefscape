package frc.robot.commands.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.swerve.Drivebase
import frc.robot.subsystems.vision.Vision
import lib.math.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.geometry.nudge
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

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
    ).apply { setTolerance(Units.inchesToMeters(0.1)) }

    private val yPid = PIDController(
        translationPID.kP,
        translationPID.kI,
        translationPID.kD
    ).apply { setTolerance(Units.inchesToMeters(0.1)) }
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
        // Re-fetch target each cycle so commands that depend on live sensors (e.g., tag-relative) work in sim/hardware
        pose = poseSupplier.get()
        Logger.recordOutput("commands/$name/Target Pose", Pose2d.struct, pose)
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
        drivebase.applyChassisSpeeds(ChassisSpeeds())
        drivebase.alignmentState = endState
    }

    companion object {
        fun nodeAlign(
            drivebase: Drivebase,
            vision: Vision,
            side: FieldConstants.Reef.Side
        ): Command {
            return Commands.idle()        }

        fun buttonBoardAlign(
            drivebase: Drivebase,
            side: FieldConstants.Reef.ReefFace,
            isL4: Trigger,
            leftSupplier: Trigger,
            rightSupplier: Trigger,
            centerSupplier: Trigger,
        ): Command {
            return AlignmentCommand(
                drivebase,
                {
                    var targetPose = FieldConstants.Reef.floorAlignmentPoses[side.ordinal].let {
                        if(AutoBuilder.shouldFlip()) it.flipped() else it
                    }

                    when {
                        leftSupplier.asBoolean
                                && !rightSupplier.asBoolean
                                && !centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                y = ((FieldConstants.Reef.sideOffset / 2.0) - manipulatorOffset).into(Meters)
                            )
                        }

                        rightSupplier.asBoolean
                                && !leftSupplier.asBoolean
                                && !centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                y = (-(FieldConstants.Reef.sideOffset / 2.0) - manipulatorOffset).into(Meters)
                            )
                        }

                        (rightSupplier.asBoolean && leftSupplier.asBoolean)
                                || centerSupplier.asBoolean -> {
                            targetPose = targetPose.nudge(
                                y = (centerOffset).into(Meters)
                            )
                        }
                    }

                    targetPose.takeIf { it.translation.getDistance(drivebase.pose.translation) <= 1.5 }
                },
                PIDGains(kP = 10.0, kD = 0.01),
                PIDGains(kP = 5.0),
                {
                    when {
                        (rightSupplier.asBoolean || leftSupplier.asBoolean) && !centerSupplier.asBoolean ->
                            Drivebase.Constants.AlignmentState.ALIGNED_CORAL
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

        // New: Tag-relative align that does not use field odometry
        fun tagRelativeAlign(
            drivebase: Drivebase,
            vision: Vision,
            standoffMeters: Double = 0.40,
            lateralOffsetMeters: Double = 0.0,
        ): Command {
            fun angleWrap(a: Double): Double {
                var x = a
                while (x > PI) x -= 2.0 * PI
                while (x < -PI) x += 2.0 * PI
                return x
            }

            return AlignmentCommand(
                drivebase,
                {
                    val allTransforms = vision.inputs.flatMap { it.targetTransforms.asList() }
                        .filter { it.fiducialId in FieldConstants.Reef.reefTags }

                    if (allTransforms.isEmpty()) return@AlignmentCommand null

                    val best = allTransforms.minBy { it.distance }
                    val perCamLatest = vision.inputs.withIndex().mapNotNull { (idx, vi) ->
                        vi.targetTransforms.filter { it.fiducialId == best.fiducialId }
                            .maxByOrNull { it.timestamp }
                            ?.let { idx to it }
                    }

                    if (perCamLatest.isEmpty()) return@AlignmentCommand null

                    var sumX = 0.0
                    var sumY = 0.0
                    var sumSin = 0.0
                    var sumCos = 0.0
                    var count = 0

                    perCamLatest.forEach { (idx, tt) ->
                        val robotToTag = Vision.robotToCameras[idx] + tt.cameraToTarget
                        sumX += robotToTag.translation.x
                        sumY += robotToTag.translation.y
                        val yaw = robotToTag.rotation.z
                        sumSin += sin(yaw)
                        sumCos += cos(yaw)
                        count++
                    }

                    val currX = sumX / count
                    val currY = sumY / count
                    val currYaw = atan2(sumSin, sumCos)

                    // Robot translation needed in robot frame to bring tag to desired standoff.
                    // Move along the line-of-sight to the tag to avoid cutting corners into the reef.
                    val bearingToTag = atan2(currY, currX)
                    val distanceToTag = kotlin.math.hypot(currX, currY)
                    val radialDelta = distanceToTag - standoffMeters
                    val deltaX = radialDelta * kotlin.math.cos(bearingToTag)
                    val deltaY = radialDelta * kotlin.math.sin(bearingToTag)
                    // Desired heading parallel to the tag face (no extra 180° flip)
                    val deltaYaw = angleWrap(currYaw)

                    // Build a "fake target" pose in field frame relative to current robot pose
                    // so existing AlignmentCommand PIDs can drive. We convert robot-frame errors
                    // into a target Pose2d in field frame by offsetting from the current pose.
                    val robotPose = drivebase.pose
                    // Small deadbands to reduce oscillation
                    val dbX = if (kotlin.math.abs(deltaX) < 0.01) 0.0 else deltaX
                    val dbY = if (kotlin.math.abs(deltaY) < 0.01) 0.0 else deltaY
                    val dbYaw = if (kotlin.math.abs(deltaYaw) < Math.toRadians(0.5)) 0.0 else deltaYaw

                    val desiredHeading = Rotation2d(robotPose.rotation.radians + dbYaw)
                    val fieldTarget = Pose2d(
                        robotPose.x + dbX * robotPose.rotation.cos - dbY * robotPose.rotation.sin,
                        robotPose.y + dbX * robotPose.rotation.sin + dbY * robotPose.rotation.cos,
                        desiredHeading
                    )

                    Logger.recordOutput("commands/TagRelativeAlign/selectedTagId", best.fiducialId)
                    Logger.recordOutput("commands/TagRelativeAlign/robotToTag/current", Transform3d.struct,
                        Transform3d(
                            edu.wpi.first.math.geometry.Translation3d(currX, currY, 0.0),
                            edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, currYaw)
                        )
                    )
                    Logger.recordOutput("commands/TagRelativeAlign/desiredPose", Pose2d.struct, fieldTarget)
                    Logger.recordOutput("commands/TagRelativeAlign/robotHeadingDeg", Math.toDegrees(robotPose.rotation.radians))
                    Logger.recordOutput("commands/TagRelativeAlign/bearingToTagDeg", Math.toDegrees(bearingToTag))
                    Logger.recordOutput("commands/TagRelativeAlign/tagYawDeg", Math.toDegrees(currYaw))
                    Logger.recordOutput("commands/TagRelativeAlign/deltaYawDeg", Math.toDegrees(dbYaw))
                    Logger.recordOutput("commands/TagRelativeAlign/desiredHeadingDeg", Math.toDegrees(desiredHeading.radians))
                    println("[TagRelAlign] tagId=${best.fiducialId} curr=(%.2f, %.2f, %.2f) bearing=%.1f tagYaw=%.1f dYaw=%.1f delta=(%.2f, %.2f) target=(%.2f, %.2f, %.1f)".format(
                        currX, currY, currYaw, Math.toDegrees(bearingToTag), Math.toDegrees(currYaw), Math.toDegrees(deltaYaw), deltaX, deltaY, fieldTarget.x, fieldTarget.y, Math.toDegrees(fieldTarget.rotation.radians)
                    ))
                    fieldTarget
                },
                translationPID = PIDGains(3.0, 0.0, 0.02),
                rotationPID = PIDGains(3.0, 0.0, 0.03),
                { Drivebase.Constants.AlignmentState.ALIGNED_CORAL }
            ).withName("TagRelativeAlign")
        }

        val leftOffset: Distance = Inches.of(-13.0)
        val rightOffset: Distance = Inches.of(-4.5)
        val centerOffset: Distance = Inches.of(-5.0)
        val backupL4: Distance = Inches.of(3.0)

        val manipulatorOffset = Inches.of(6.0)

        val alignLimits: PathConstraints = PathConstraints(
            MetersPerSecond.of(1.5),
            MetersPerSecondPerSecond.of(8.0),
            DegreesPerSecond.of(360 + 180.0),
            DegreesPerSecondPerSecond.of(720.0)
        )
    }


}