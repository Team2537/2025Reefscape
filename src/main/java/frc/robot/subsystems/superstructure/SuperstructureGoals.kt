package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import java.nio.ByteBuffer
import java.util.Optional

object SuperstructureGoals {
    val stowAngle = Degrees.of(90.0)
    val scoreAngle = Degrees.of(35.0)

    val STOW = SuperstructureState(
        armAngle = Degrees.of(16.0),
        elevatorHeight = Inches.of(0.0),
        driveLimits = Drivebase.defaultLimits,
        name = "Stow"
    )

    val ALGAE_L2 = SuperstructureState(
        armAngle = Degrees.of(40.0),
        elevatorHeight = Inches.of(14.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Algae L2"
    )

    val ALGAE_L3 = SuperstructureState(
        armAngle = Degrees.of(180.0),
        elevatorHeight = Inches.of(31.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Algae L3"
    )
    
    val L1 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inches.of(1.15),
        driveLimits = Drivebase.extendedLimits,
        name = "L1"
    )
    
    val L2 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(9.5),
        driveLimits = Drivebase.extendedLimits,
        name = "L2"
    )
    
    val L3 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(24.5),
        driveLimits = Drivebase.extendedLimits,
        name = "L3"
    )
    
    val L4 = SuperstructureState(
        armAngle = Degrees.of(55.0),
        elevatorHeight = Inches.of(50.0),
        driveLimits = Drivebase.extendedLimits,
        name = "L4"
    )
}