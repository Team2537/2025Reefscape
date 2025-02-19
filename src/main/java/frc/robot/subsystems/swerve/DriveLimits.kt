package frc.robot.subsystems.swerve

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.superstructure.SuperstructureGoal.DriveLimitStruct

data class DriveLimits(
    val maxLinVel: LinearVelocity,
    val maxAngVel: AngularVelocity,
    val maxAccel: LinearAcceleration
) : StructSerializable {
    companion object {
        val struct = DriveLimitStruct()
    }
}