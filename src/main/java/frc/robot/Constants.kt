package frc.robot

import lib.math.units.lbs

object Constants {
    val robotMass = when(RobotType.type) {
        RobotType.Type.ROBOT_2025_COMP -> 125.0.lbs
        RobotType.Type.ROBOT_2025_SWERVE_BASE -> 50.0.lbs
    }
}