package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.RadiansPerSecond
import java.util.function.Supplier

class GyroIOSim(
    private val speeds: Supplier<ChassisSpeeds>
) : GyroIO {
    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.yaw += Rotation2d.fromRadians(speeds.get().omegaRadiansPerSecond * 0.02)
        inputs.yawRate.mut_replace(speeds.get().omegaRadiansPerSecond * 0.02, RadiansPerSecond)
        
        inputs.fullRotation = Rotation3d(inputs.yaw)
    }
}