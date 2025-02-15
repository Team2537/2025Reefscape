package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.swerve.gyro.GyroIO
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOMapleSim(val gyroSimulation: GyroSimulation): GyroIO {
    /**
     * Method for pulling data from the gyro. Updates the GyroInputs object.
     */
    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.isConnected = true
        inputs.yaw = gyroSimulation.gyroReading
        inputs.yawRate.mut_replace(gyroSimulation.measuredAngularVelocity)
    }
    
    override fun setYaw(heading: Rotation2d) {
        super.setYaw(heading)
    }
}