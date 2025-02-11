package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface VisionIO {
    class VisionInputs : LoggableInputs {
        var isConnected: Boolean = false
        var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d(), Rotation2d())
        var poseObservations: Array<PoseObservation> = emptyArray()
        var tagIDs = IntArray(0)

        override fun toLog(table: LogTable) {
            table.put("isConnected", isConnected)
            table.put("latestTargetObservation", latestTargetObservation)
            table.put("poseObservations", *poseObservations)
            table.put("tagIDs", tagIDs)
       }

        override fun fromLog(table: LogTable) {
            isConnected = table.get("isConnected", isConnected)
            latestTargetObservation = table.get("latestTargetObservation", latestTargetObservation)
            poseObservations = table.get("poseObservations", *poseObservations)
            tagIDs = table.get("tagIDs", tagIDs)
        }
    }

    @JvmRecord
    data class TargetObservation(val tx: Rotation2d, val ty: Rotation2d)

    @JvmRecord
    data class PoseObservation(
        val timestamp: Double,
        val pose: Pose3d,
        val ambiguity: Double,
        val tagCount: Int,
        val averageTagDistance: Double,
    )

    fun updateInputs(inputs: VisionInputs) {}
}