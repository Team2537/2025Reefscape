package frc.robot.subsystems.vision

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.Filesystem
import lib.math.geometry.FieldConstants
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

class VisionIOPhotonVisionSim(
    name: String,
    robotToCamera: Transform3d,
    val poseSupplier: Supplier<Pose2d>
) : VisionIOPhotonVision(name, robotToCamera) {
    
    
    
    private val cameraProperties: SimCameraProperties = SimCameraProperties()
        .apply {
            setCalibration(
                640, 480,
                MatBuilder.fill(
                    Nat.N3(), Nat.N3(),
                    546.947202769191, 0.0, 317.2326216443899,
                    0.0, 546.8805910328873, 256.54365866088693,
                    0.0, 0.0, 1.0
                ),
                MatBuilder.fill(
                    Nat.N8(), Nat.N1(),
                    0.04575330800554877,
                    -0.06764388351284431,
                    -0.0002716072733319333,
                    -0.0008388770763606548,
                    0.007416750430854674,
                    -0.0016856447818708886,
                    0.0027323859252879066,
                    -0.00029965498573175946
                )
            )
            
            setCalibError(0.34, 0.05)
        }
    
    private val cameraSim = PhotonCameraSim(camera, cameraProperties)
        .also { visionSim.addCamera(it, robotToCamera) }
    
    
    override fun updateInputs(inputs: VisionIO.VisionInputs) {
        visionSim.update(poseSupplier.get())
        super.updateInputs(inputs)
    }
    
    companion object {
        private val visionSim: VisionSystemSim =
            VisionSystemSim("main")
                .apply { addAprilTags(FieldConstants.tagLayout) }
    }
}