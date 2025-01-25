package frc.robot.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Vision : SubsystemBase("vision") {
    val io: List<VisionIO> =
        listOf(
            VisionIOPhotonVision("front_left_cam", Transform3d()),
            VisionIOPhotonVision("front_right_cam", Transform3d()),
        )

    val inputs: List<VisionIO.VisionInputs> = io.map { VisionIO.VisionInputs() }

    override fun periodic() {
        io.zip(inputs).forEachIndexed { index, (io, inputs) ->
            io.updateInputs(inputs)
            Logger.processInputs("vision/Camera $index", inputs)
        }

        val allTagPoses: MutableList<Pose3d> = mutableListOf()
        val allRobotPoses: MutableList<Pose3d> = mutableListOf()
        val allAcceptedRobotPoses: MutableList<Pose3d> = mutableListOf()
        val allRejectedRobotPoses: MutableList<Pose3d> = mutableListOf()
    }

    @FunctionalInterface
    interface VisionConsumer {
        fun accept(
            pose: Pose2d,
            timestamp: Double,
            stdDevs: Matrix<N3, N1>
        )
    }
}