package frc.robot.subsystems.vision

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotType
import lib.math.geometry.FieldConstants
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.pow

typealias VisionConsumer = (Pose2d, Double, Vector<N3>) -> Unit

class Vision(val consumer: VisionConsumer) : SubsystemBase("vision") {
    
    val robotPoseSupplier: Supplier<Pose2d> = Supplier { Robot.drivebase.pose }
    
    val io: List<VisionIO> =
        when (RobotType.mode) {
//            RobotType.Mode.REAL -> listOf(
//                VisionIOPhotonVision("front_left_cam", robotToCameras[0]),
//                VisionIOPhotonVision("front_right_cam", robotToCameras[1]),
//            )
//
//            RobotType.Mode.SIMULATION -> listOf(
//                VisionIOPhotonVisionSim(
//                    "front_left_cam", robotToCameras[0], robotPoseSupplier
//                ),
//                VisionIOPhotonVisionSim(
//                    "front_right_cam", robotToCameras[1], robotPoseSupplier
//                ),
//                VisionIOPhotonVisionSim(
//                    "back_left_cam", robotToCameras[2], robotPoseSupplier
//                ),
//                VisionIOPhotonVisionSim(
//                    "back_right_cam", robotToCameras[3], robotPoseSupplier
//                )
//            )
            
            else -> listOf(
                object : VisionIO {},
                object : VisionIO {},
            )
        }
    
    val inputs: List<VisionIO.VisionInputs> = io.map { VisionIO.VisionInputs() }
    
    
    override fun periodic() {
        io.zip(inputs).forEachIndexed { index, (io, inputs) ->
            io.updateInputs(inputs)
            Logger.processInputs("$name/Camera$index", inputs)
        }
        
        Logger.recordOutput(
            "cameraPoses",
            *robotToCameras.map { Pose3d(robotPoseSupplier.get()).transformBy(it) }.toTypedArray()
        )
        
        val allTagPoses: MutableList<Pose3d> = mutableListOf()
        val allRobotPoses: MutableList<Pose3d> = mutableListOf()
        val allAcceptedRobotPoses: MutableList<Pose3d> = mutableListOf()
        val allRejectedRobotPoses: MutableList<Pose3d> = mutableListOf()
        
        inputs.forEachIndexed { index, inputs ->
            val tagPoses: MutableList<Pose3d> = mutableListOf()
            val robotPoses: MutableList<Pose3d> = mutableListOf()
            val acceptedRobotPoses: MutableList<Pose3d> = mutableListOf()
            val rejectedRobotPoses: MutableList<Pose3d> = mutableListOf()
            
            inputs.tagIDs.forEach { id ->
                val pose = FieldConstants.tagLayout.getTagPose(id)
                if (pose.isPresent) {
                    tagPoses.add(pose.get())
                }
            }
            
            for (observation in inputs.poseObservations) {
                val rejectPose: Boolean =
                    observation.tagCount == 0
                      || (observation.tagCount == 1 && observation.ambiguity > maxAmbiguity)
                      || abs(observation.pose.z) > maxZError
                      || observation.pose.x < 0.0
                      || observation.pose.x > FieldConstants.tagLayout.fieldLength
                      || observation.pose.y < 0.0
                      || observation.pose.y > FieldConstants.tagLayout.fieldWidth
                
                robotPoses.add(observation.pose)
                if (rejectPose) {
                    rejectedRobotPoses.add(observation.pose)
                    continue
                } else {
                    acceptedRobotPoses.add(observation.pose)
                }
                
                val stdDevFactor =
                    observation.averageTagDistance.pow(2) / observation.tagCount
                
                val linearStdDev = linearStdDevBaseline * stdDevFactor
                val angularStdDev = angularStdDevBaseline * stdDevFactor
                
                consumer(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    VecBuilder.fill(
                        linearStdDev,
                        linearStdDev,
                        angularStdDev
                    )
                )
            }
            
            Logger.recordOutput("$name/Camera$index/TagPoses", *tagPoses.toTypedArray())
            Logger.recordOutput("$name/Camera$index/RobotPoses", *robotPoses.toTypedArray())
            Logger.recordOutput("$name/Camera$index/AcceptedRobotPoses", *acceptedRobotPoses.toTypedArray())
            Logger.recordOutput("$name/Camera$index/RejectedRobotPoses", *rejectedRobotPoses.toTypedArray())
            
            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(robotPoses)
            allAcceptedRobotPoses.addAll(acceptedRobotPoses)
            allRejectedRobotPoses.addAll(rejectedRobotPoses)
        }
        
        Logger.recordOutput("$name/Summary/TagPoses", *allTagPoses.toTypedArray())
        Logger.recordOutput("$name/Summary/RobotPoses", *allRobotPoses.toTypedArray())
        Logger.recordOutput("$name/Summary/AcceptedRobotPoses", *allAcceptedRobotPoses.toTypedArray())
        Logger.recordOutput("$name/Summary/RejectedRobotPoses", *allRejectedRobotPoses.toTypedArray())
    }
    
    companion object {
        val robotToCameras = listOf(
            Transform3d(
                Translation3d(Inches.of(12.875), Inches.of(-11.875), Inches.of(6.883276 + 1.5)),
                Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(25.0))
            ),
            Transform3d(
                Translation3d(Inches.of(9.75), Inches.of(2.015), Inches.of(16.665)),
                Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-20.0))
            ),
            Transform3d(
                Translation3d(Inches.of(-12.875), Inches.of(-11.875), Inches.of(6.883276 + 1.5)),
                Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(180.0 - 15.0))
            ),
            Transform3d(
                Translation3d(Inches.of(-9.75), Inches.of(2.015), Inches.of(6.883276 + 1.5)),
                Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians( 180 - 15.0))
            ),
        )
        
        // Basic filtering thresholds
        val maxAmbiguity: Double = 0.3
        val maxZError: Double = 0.75
        
        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        val linearStdDevBaseline: Double = 0.02 // Meters
        val angularStdDevBaseline: Double = 0.06 // Radians
    }
    
}