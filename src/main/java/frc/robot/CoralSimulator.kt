package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import lib.math.geometry.FieldConstants
import lib.math.geometry.to3d
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class CoralSimulator(
    private val armPoseSupplier: Supplier<Pose3d>,
    private val robotPoseSupplier: Supplier<Pose2d>,
    private val armToGripperCenter: Distance
) {
    
    private val nodes = FieldConstants.Reef.nodes
    
    private val scoredNodes: MutableSet<FieldConstants.Reef.Node> = mutableSetOf()
    
    fun score() {
        // Calculate the arm pose in 3D space
        val armPose: Pose3d = robotPoseSupplier.get().to3d.transformBy(
            Transform3d(
                armPoseSupplier.get().translation,
                armPoseSupplier.get().rotation
            )
        )
        
        // Calculate the gripper pose relative to the arm
        val gripperPose = armPose.transformBy(
            Transform3d(
                Translation3d(armToGripperCenter, Inches.of(-6.0), Meters.zero()),
                Rotation3d()
            )
        )
        // Variables to track the closest point
        var closestPoint: FieldConstants.Reef.Node? = null
        var minDistance = Double.MAX_VALUE
        
        // Iterate over all points to find the closest one
        for (point in nodes) {
            // Calculate the Euclidean distance to the gripper pose's position
            val distance = point.pose.translation.getDistance(gripperPose.translation)
            
            // Update the closest point if this one is nearer
            if (distance < minDistance) {
                minDistance = distance
                closestPoint = point
            }
        }
        
        
        if(closestPoint != null && minDistance < Units.inchesToMeters(12.0)) {
            scoredNodes.add(closestPoint)
        }
        
    }
    
    fun update() {
        Logger.recordOutput("scoredNodes", *scoredNodes.map { it.pose }.toTypedArray())
    }
    
}