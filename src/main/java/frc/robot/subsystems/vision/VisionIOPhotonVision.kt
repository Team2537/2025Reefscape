package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import lib.math.geometry.FieldConstants
import org.photonvision.PhotonCamera

open class VisionIOPhotonVision(name: String, val robotToCamera: Transform3d) : VisionIO {
    val camera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIO.VisionInputs) {
        inputs.isConnected = camera.isConnected

        var tagIDs: MutableSet<Short> = mutableSetOf()
        val poseObservations: MutableList<VisionIO.PoseObservation> = mutableListOf()

        camera.allUnreadResults.forEach { result ->
            if(result.hasTargets()){
                inputs.latestTargetObservation =
                    VisionIO.TargetObservation(
                        Rotation2d.fromDegrees(result.bestTarget.yaw),
                        Rotation2d.fromDegrees(result.bestTarget.pitch)
                    )
            } else {
                inputs.latestTargetObservation = VisionIO.TargetObservation(Rotation2d(), Rotation2d())
            }

            if(result.multitagResult.isPresent) {
                val multitagResult = result.multitagResult.get()
                val fieldToCamera: Transform3d = multitagResult.estimatedPose.best
                val fieldToRobot = fieldToCamera + robotToCamera.inverse()

                val robotPose = Pose3d(
                    fieldToRobot.translation,
                    fieldToRobot.rotation
                )

                val totalTagDistance = result.targets.sumOf { it.bestCameraToTarget.translation.norm }

                tagIDs.addAll(multitagResult.fiducialIDsUsed)

                poseObservations.add(
                    VisionIO.PoseObservation(
                        result.timestampSeconds,
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size,
                        totalTagDistance / result.targets.size,
                    )
                )
            } else if (result.targets.isNotEmpty()) {
                val target = result.targets.first()
                val tagPose = FieldConstants.tagLayout.getTagPose(target.fiducialId)
                if(tagPose.isPresent) {
                    val fieldToTarget = Transform3d(
                        tagPose.get().translation,
                        tagPose.get().rotation
                    )

                    val cameraToTarget = target.bestCameraToTarget
                    val fieldToCamera = fieldToTarget + cameraToTarget.inverse()
                    val fieldToRobot = fieldToCamera + robotToCamera.inverse()
                    val robotPose = Pose3d(
                        fieldToRobot.translation,
                        fieldToRobot.rotation
                    )

                    tagIDs.add(target.fiducialId.toShort())
                    poseObservations.add(
                        VisionIO.PoseObservation(
                            result.timestampSeconds,
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.translation.norm,
                        )
                    )
                }
            }
        }

        inputs.poseObservations = poseObservations.toTypedArray()
        inputs.tagIDs = IntArray(tagIDs.size) { tagIDs.elementAt(it).toInt() }
    }
}