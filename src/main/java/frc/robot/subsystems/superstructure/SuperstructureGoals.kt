package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.swerve.DriveLimits
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import java.nio.ByteBuffer
import java.util.Optional

object SuperstructureGoals {
    val stowAngle = Degrees.of(90.0)
    val scoreAngle = Degrees.of(65.0)

    val STOW = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(2.75),
        driveLimits = Drivebase.defaultLimits
    )
    
    val SOURCE = SuperstructureState(
        armAngle = Degrees.of(-135.0),
        elevatorHeight = Meters.of(1.0),
        driveLimits = Drivebase.defaultLimits
    )
    
    val PRE_SOURCE = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.5),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L1 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inches.of(1.15),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L2 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(6.0),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L3 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(22.0),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L4 = SuperstructureState(
        armAngle = Degrees.of(60.0),
        elevatorHeight = Inches.of(47.0),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L1_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(1.15),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L1)
    )
    
    val L2_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(6.0),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L2)
    )
    
    val L3_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(22.0),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L3)
    )
    
    val L4_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(49.0),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L4)
    )
}