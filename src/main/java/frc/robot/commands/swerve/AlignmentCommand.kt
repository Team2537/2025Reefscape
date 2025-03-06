package frc.robot.commands.swerve

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.subsystems.swerve.Drivebase
import lib.controllers.gains.PIDGains
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.units.inches
import lib.math.units.into
import java.util.*
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI

class AlignmentCommand(
    val drivebase: Drivebase,
    val poseSupplier: Supplier<Pose2d?>,
    val translationPID: PIDGains,
    val rotationPID: PIDGains
) : Command() {
    private val xPid = PIDController(translationPID.kP, translationPID.kI, translationPID.kD)
    private val yPid = PIDController(translationPID.kP, translationPID.kI, translationPID.kD)
    private val rotPid =
        PIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD).apply { enableContinuousInput(0.0, 2 * PI) }
    
    private var pose: Pose2d? = null
    
    init {
        addRequirements(drivebase)
    }
    
    override fun initialize() {
        xPid.reset()
        yPid.reset()
        rotPid.reset()
        
        pose = poseSupplier.get()
    }
    
    override fun execute() {
        pose?.let {
            val translation = drivebase.pose.translation
            val rotation = drivebase.pose.rotation
            
            val xOutput = xPid.calculate(translation.x, pose!!.translation.x)
            val yOutput = yPid.calculate(translation.y, pose!!.translation.y)
            val rotOutput = rotPid.calculate(rotation.radians, pose!!.rotation.radians)
            
            drivebase.applyChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, rotOutput, rotation))
        } ?: return
    }
    
    override fun isFinished(): Boolean {
        return (xPid.atSetpoint() && yPid.atSetpoint() && rotPid.atSetpoint()) || pose == null
    }
    
    companion object {
        fun nodeAlignment(
            drivebase: Drivebase,
            side: FieldConstants.Reef.Side,
            coralDistanceSupplier: Supplier<Distance>
        ): Command {
            return AlignmentCommand(
                drivebase = drivebase,
                poseSupplier = {
                    val currPose = drivebase.pose.let {
                        if (AutoBuilder.shouldFlip()) it.flipped() else it
                    }
                    
                    val closestReefSide = currPose.nearest(FieldConstants.Reef.floorAlignmentPoses)
                    
                    val baseOffset = if (side == FieldConstants.Reef.Side.LEFT) 6.0.inches else 3.5.inches
                    
                    val sideOffsetDistance =
                        (FieldConstants.Reef.sideOffset / (if (side == FieldConstants.Reef.Side.LEFT) 2.0 else -2.0)) + baseOffset + coralDistanceSupplier.get()
                    
                    val targetPose = closestReefSide.transformBy(
                        Transform2d(
                            Translation2d(0.0, sideOffsetDistance into Meters),
                            Rotation2d()
                        )
                    ).let { if (AutoBuilder.shouldFlip()) it.flipped() else it }
                    
                    val distanceToTarget = targetPose.translation.getDistance(drivebase.pose.translation)
                    
                    targetPose.takeIf { distanceToTarget <= 1.5 }
                },
                translationPID = PIDGains(5.0, 0.0, 0.05),
                rotationPID = PIDGains(5.0, 0.0, 0.0)
            ).withName("NodeAlignmentCommand(${side.name})")
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
            ).withName("AlgaeAlignmentCommand")
        }
        
        fun sourceAlignment(
            drivebase: Drivebase,
        ): Command {
            return AlignmentCommand(
                drivebase,
                {
                    val currPose = drivebase.pose.let { if (AutoBuilder.shouldFlip()) it.flipped() else it }
                    val targetPose = currPose.nearest(FieldConstants.Reef.floorAlignmentPoses)
                        .let { if (AutoBuilder.shouldFlip()) it.flipped() else it }
                    
                    targetPose.takeIf { it.translation.getDistance(drivebase.pose.translation) >= 1.0 }
                },
                translationPID = PIDGains(5.0, 0.0, 0.05),
                rotationPID = PIDGains(0.5, 0.0, 0.0)
            ).withName("SourceAlignmentCommand")
        }
    }
}