package frc.robot.commands.swerve

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.Drivebase
import lib.math.geometry.FieldConstants
import lib.math.geometry.flipped
import lib.math.units.into

class NodeAlignmentCommand(val drivebase: Drivebase, val side: FieldConstants.Reef.Side): Command() {

    init {
        addRequirements(drivebase)
    }

    val xPid = PIDController(5.0, 0.0, 0.05)
    val yPid = PIDController(5.0, 0.0, 0.05)
    val anglePid = PIDController(3.0, 0.0, 0.05)

    var endPose: Pose2d? = null

    override fun initialize() {
        val currentPose = if(AutoBuilder.shouldFlip()) { drivebase.pose.flipped() } else { drivebase.pose }

        val closestPose = currentPose.nearest(FieldConstants.Reef.floorAlignmentPoses)

        val sideOffsetDistance = if(side == FieldConstants.Reef.Side.LEFT) {
            FieldConstants.Reef.sideOffset / 2.0
        } else {
            FieldConstants.Reef.sideOffset / -2.0
        }

        val sideOffset = Translation2d(0.0, (sideOffsetDistance) into Meters)

        var targetPose = closestPose.transformBy(Transform2d(sideOffset, Rotation2d()))

        if(AutoBuilder.shouldFlip()) { targetPose = targetPose.flipped() }

        endPose = targetPose

        xPid.reset()
        yPid.reset()
        anglePid.reset()
    }

    override fun execute() {
        if(endPose == null) return
        drivebase.applyChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xPid.calculate(drivebase.pose.translation.x, endPose!!.translation.x),
                yPid.calculate(drivebase.pose.translation.y, endPose!!.translation.y),
                anglePid.calculate(drivebase.pose.rotation.radians, endPose!!.rotation.radians),
                drivebase.gyroInputs.yaw
            )
        )
    }

    override fun isFinished(): Boolean {
        return (xPid.atSetpoint() && yPid.atSetpoint() && anglePid.atSetpoint()) || endPose == null
    }

    override fun end(interrupted: Boolean) {
        drivebase.applyChassisSpeeds(ChassisSpeeds())
    }
}