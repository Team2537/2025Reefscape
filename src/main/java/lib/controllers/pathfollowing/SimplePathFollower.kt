package lib.controllers.pathfollowing

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.subsystems.swerve.Drivebase
import lib.controllers.gains.PIDGains
import org.littletonrobotics.junction.Logger
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.PI

class SimplePathFollower(
    private val drivebase: Drivebase,
    translationPIDGains: PIDGains,
    thetaPidGains: PIDGains,
    private val speedConsumer: Consumer<ChassisSpeeds>,
    private val poseSupplier: Supplier<Pose2d>
) : PathFollower {

    private val xPID = PIDController(translationPIDGains.kP, translationPIDGains.kI, translationPIDGains.kD)
    private val yPID = PIDController(translationPIDGains.kP, translationPIDGains.kI, translationPIDGains.kD)
    private val thetaPID = PIDController(thetaPidGains.kP, thetaPidGains.kI, thetaPidGains.kD)
        .apply { enableContinuousInput(0.0, 2 * PI) }

    override fun accept(sample: SwerveSample) {
        val pose = poseSupplier.get()

        val xOutput = xPID.calculate(pose.translation.x, sample.x)
        val yOutput = yPID.calculate(pose.translation.y, sample.y)
        val thetaOutput = thetaPID.calculate(pose.rotation.radians, sample.heading)

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput + sample.vx,
            yOutput + sample.vy,
            thetaOutput + sample.omega,
            pose.rotation
        )


        Logger.recordOutput("drivebase/auto/xError", xPID.error)
        Logger.recordOutput("drivebase/auto/yError", yPID.error)
        Logger.recordOutput("drivebase/auto/thetaError", thetaPID.error)

        Logger.recordOutput("drivebase/auto/samplePose", Pose2d.struct, sample.pose)
        Logger.recordOutput("drivebase/auto/pose", Pose2d.struct, pose)

        speedConsumer.accept(speeds)
    }
}