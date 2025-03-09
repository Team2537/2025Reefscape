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
    val scoreAngle = Degrees.of(70.0)

    val STOW = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(2.75),
        driveLimits = Drivebase.defaultLimits
    )
    
    val SOURCE = SuperstructureState(
        armAngle = Degrees.of(-135.0),
        elevatorHeight = Inches.of(40.0),
        driveLimits = Drivebase.intakeLimits
    )
    
    val PRE_SOURCE = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.5),
        driveLimits = Drivebase.extendedLimits,
    )

    val ALGAE_L2 = SuperstructureState(
        armAngle = Degrees.of(40.0),
        elevatorHeight = Inches.of(14.0),
        driveLimits = Drivebase.extendedLimits,
    )

    val ALGAE_L3 = SuperstructureState(
        armAngle = Degrees.of(50.0),
        elevatorHeight = Inches.of(25.0),
        driveLimits = Drivebase.extendedLimits,
    )

    val PREP_ALGAE_L2 = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(16.0),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(ALGAE_L2)
    )


    val PREP_ALGAE_L3 = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(10.0),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(ALGAE_L3)
    )
    
    val L1 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inches.of(1.15),
        driveLimits = Drivebase.extendedLimits,
    )
    
    val L2 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(7.0),
        driveLimits = Drivebase.extendedLimits,
    )
    
    val L3 = SuperstructureState(
        armAngle = scoreAngle,
        elevatorHeight = Inch.of(22.0),
        driveLimits = Drivebase.extendedLimits,
    )
    
    val L4 = SuperstructureState(
        armAngle = Degrees.of(35.0),
        elevatorHeight = Inches.of(51.0),
        driveLimits = Drivebase.extendedLimits,
    )

    val CLIMB = SuperstructureState(
        armAngle = Degrees.of(180.0),
        elevatorHeight = Inches.of(3.0),
        driveLimits = Drivebase.slowmodeLimits
    )

    val PRE_CLIMB = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(6.0),
        driveLimits = Drivebase.slowmodeLimits,
        nextState = Optional.of(CLIMB)
    )
    
    val L1_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(1.15),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(L1)
    )
    
    val L2_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(8.0),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(L2)
    )
    
    val L3_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(22.0),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(L3)
    )
    
    val L4_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(51.0),
        driveLimits = Drivebase.extendedLimits,
        nextState = Optional.of(L4)
    )
}