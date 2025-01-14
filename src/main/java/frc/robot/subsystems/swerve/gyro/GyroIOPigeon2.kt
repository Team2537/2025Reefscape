package frc.robot.subsystems.swerve.gyro

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d

class GyroIOPigeon2(val id: Int): GyroIO {
    private val pigeon: Pigeon2 = Pigeon2(id).apply {
        val config = Pigeon2Configuration()
    }

    val yaw = pigeon.yaw.clone()
    val pitch = pigeon.pitch.clone()
    val roll = pigeon.roll.clone()

    val yawRate = pigeon.angularVelocityZWorld.clone()
    val pitchRate = pigeon.angularVelocityYWorld.clone()
    val rollRate = pigeon.angularVelocityXWorld.clone()

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.yaw = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputs.pitch = Rotation2d.fromDegrees(pitch.valueAsDouble)
        inputs.roll = Rotation2d.fromDegrees(roll.valueAsDouble)

        inputs.yawRate.mut_replace(yawRate.value)
        inputs.pitchRate.mut_replace(pitchRate.value)
        inputs.rollRate.mut_replace(rollRate.value)
    }
}