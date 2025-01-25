package lib.controllers.pathfollowing

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N2
import frc.robot.subsystems.swerve.Drivebase
import lib.controllers.gains.PIDGains
import org.littletonrobotics.junction.Logger
import java.util.function.BiConsumer
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.PI

class ModuleForcesPathFollower(
    private val drivebase: Drivebase,
    xPidGains: PIDGains,
    yPidGains: PIDGains,
    thetaPidGains: PIDGains,
    private val speedConsumer: BiConsumer<ChassisSpeeds, List<Vector<N2>>>,
    private val poseSupplier: Supplier<Pose2d>
): PathFollower {

    private val xPID = PIDController(xPidGains.kP, xPidGains.kI, xPidGains.kD)
    private val yPID = PIDController(yPidGains.kP, yPidGains.kI, yPidGains.kD)
    private val thetaPID = PIDController(thetaPidGains.kP, thetaPidGains.kI, thetaPidGains.kD)
        .apply { enableContinuousInput(-PI, PI) }

    override fun accept(sample: SwerveSample) {
        val pose = poseSupplier.get()

        val xOutput = xPID.calculate(pose.translation.x, sample.x)
        val yOutput = yPID.calculate(pose.translation.y, sample.y)
        val thetaOutput = thetaPID.calculate(pose.rotation.radians, sample.heading)

        Logger.recordOutput("drivebase/auto/xError", xPID.error)
        Logger.recordOutput("drivebase/auto/yError", yPID.error)
        Logger.recordOutput("drivebase/auto/thetaError", thetaPID.error)

        Logger.recordOutput("drivebase/auto/samplePose", Pose2d.struct, sample.pose)
        Logger.recordOutput("drivebase/auto/pose", Pose2d.struct, pose)

        speedConsumer.accept(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xOutput + sample.vx,
                yOutput + sample.vy,
                thetaOutput + sample.omega,
                pose.rotation
            ),
            sample.moduleForcesX().zip(sample.moduleForcesY()).map { VecBuilder.fill(it.first, it.second) }
        )
    }
}