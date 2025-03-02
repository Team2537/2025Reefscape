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
import lib.math.units.inches
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.PI

class NodeAlignmentCommand(val drivebase: Drivebase, val side: FieldConstants.Reef.Side): Command() {

    init {
        addRequirements(drivebase)
    }

    val xPid = PIDController(5.0, 0.0, 0.05)
    val yPid = PIDController(5.0, 0.0, 0.05)
    val anglePid = PIDController(1.0, 0.0, 0.05).apply { enableContinuousInput(0.0, 2 * PI) }

    var endPose: Pose2d? = null

    override fun initialize() {
        endPose = null

        val currentPose = if(AutoBuilder.shouldFlip()) { drivebase.pose.flipped() } else { drivebase.pose }

        val closestPose = currentPose.nearest(FieldConstants.Reef.floorAlignmentPoses)

        val sideOffsetDistance = if(side == FieldConstants.Reef.Side.LEFT) {
            (FieldConstants.Reef.sideOffset / 2.0) + 6.0.inches
        } else {
            (FieldConstants.Reef.sideOffset / -2.0) + 7.0.inches
        }

        val sideOffset = Translation2d(0.0, (sideOffsetDistance) into Meters)

        var targetPose = closestPose.transformBy(Transform2d(sideOffset, Rotation2d()))

        if(AutoBuilder.shouldFlip()) { targetPose = targetPose.flipped() }

        if(targetPose.translation.getDistance(drivebase.pose.translation) < 1.0) {
            endPose = targetPose
        }

        xPid.reset()
        yPid.reset()
        anglePid.reset()
    }

    override fun execute() {
        if(endPose == null) return else Logger.recordOutput("commands/$name/endPose", endPose!!)

        val xOutput = xPid.calculate(drivebase.pose.translation.x, endPose!!.translation.x)
        val yOutput = yPid.calculate(drivebase.pose.translation.y, endPose!!.translation.y)
        val angleOutput = anglePid.calculate(drivebase.pose.rotation.radians, endPose!!.rotation.radians)

        Logger.recordOutput("commands/$name/xOutput", xOutput)
        Logger.recordOutput("commands/$name/yOutput", yOutput)
        Logger.recordOutput("commands/$name/angleOutput", angleOutput)

        drivebase.applyChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xOutput,
                yOutput,
                angleOutput,
                drivebase.pose.rotation + drivebase.operatorPerspective
            )
        )
    }

    override fun isFinished(): Boolean {
        println("xPid: ${xPid.atSetpoint()} yPid: ${yPid.atSetpoint()} anglePid: ${anglePid.atSetpoint()} Endpose: ${endPose == null}")
        return (xPid.atSetpoint() && yPid.atSetpoint() && anglePid.atSetpoint()) || endPose == null
    }

    override fun end(interrupted: Boolean) {
        drivebase.applyChassisSpeeds(ChassisSpeeds())
        println(interrupted)
    }
}