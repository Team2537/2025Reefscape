package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity

object SuperstructureGoal {
    data class SuperstructureState(
        val armAngle: Angle,
        val elevatorHeight: Distance,
        val driveLimits: DriveLimits
    )
    
    data class DriveLimits(
        val maxLinVel: LinearVelocity,
        val maxAngVel: AngularVelocity,
        val maxAccel: LinearAcceleration
    )
    
    val STOW = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(2.75),
        driveLimits = DriveLimits(
            maxLinVel = LinearVelocity(0.0),
            maxAngVel = AngularVelocity(0.0),
            maxAccel = LinearAcceleration(0.0)
        )
    )
}