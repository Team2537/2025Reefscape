package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
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
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.028),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L2 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.238),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L3 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.642),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L4 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(1.289),
        driveLimits = Drivebase.defaultLimits,
    )
    
    val L1_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.038),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L1)
    )
    
    val L2_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.248),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L2)
    )
    
    val L3_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.652),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L3)
    )
    
    val L4_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(1.299),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L4)
    )
}