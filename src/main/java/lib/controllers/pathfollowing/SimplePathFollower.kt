package lib.controllers.pathfollowing

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.subsystems.swerve.Drivebase
import lib.controllers.gains.PIDGains
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.PI

class SimplePathFollower(
    private val drivebase: Drivebase,
    xPidGains: PIDGains,
    yPidGains: PIDGains,
    thetaPidGains: PIDGains,
    private val speedConsumer: Consumer<ChassisSpeeds>,
    private val poseSupplier: Supplier<Pose2d>
): Consumer<SwerveSample> {

    private val xPID = PIDController(xPidGains.kP, xPidGains.kI, xPidGains.kD)
    private val yPID = PIDController(yPidGains.kP, yPidGains.kI, yPidGains.kD)
    private val thetaPID = PIDController(thetaPidGains.kP, thetaPidGains.kI, thetaPidGains.kD)
        .apply { enableContinuousInput(-PI, PI) }

    override fun accept(sample: SwerveSample) {
        val pose = poseSupplier.get()

        val xOutput = xPID.calculate(pose.translation.x, sample.x)
        val yOutput = yPID.calculate(pose.translation.y, sample.y)
        val thetaOutput = thetaPID.calculate(pose.rotation.radians, sample.heading)

        speedConsumer.accept(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xOutput + sample.vx,
                yOutput + sample.vy,
                thetaOutput + sample.omega,
                pose.rotation
            )
        )
    }
}