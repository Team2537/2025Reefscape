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

// TODO: these need to be tuned significantly
object SuperstructureGoals {
    val STOW = SuperstructureState(
        armAngle = Degrees.of(63.0),
        driveLimits = Drivebase.defaultLimits,
        name = "Stow"
    )

    // val ALGAE_STOW = SuperstructureState(
    //     armAngle = Degrees.of(-20.0),
    //     driveLimits = Drivebase.defaultLimits,
    //     name = "Algae Stow"
    // )

    val PRE_ALGAE_L2 = SuperstructureState(
        armAngle = Degrees.of(63.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Pre Algae L2"
    )

    val PRE_ALGAE_L3 = SuperstructureState(
        armAngle = Degrees.of(125.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Pre Algae L3"
    )

    val ALGAE_L2 = SuperstructureState(
        armAngle = Degrees.of(34.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Algae L2"
    )

    val ALGAE_L3 = SuperstructureState(
        armAngle = Degrees.of(125.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Algae L3"
    )

    // val BARGE = SuperstructureState(
    //     armAngle = Degrees.of(-20.0),
    //     driveLimits = Drivebase.extendedLimits,
    //     name = "Barge"
    // )
    
    val L1 = SuperstructureState(
        armAngle = Degrees.of(34.0), // needs tuning
        driveLimits = Drivebase.extendedLimits,
        name = "L1"
    )

    val GROUND = SuperstructureState(
        armAngle = Degrees.of(1.0),
        driveLimits = Drivebase.extendedLimits,
        name = "Ground"
    )
    
    // val L2 = SuperstructureState(
    //     armAngle = scoreAngle,
    //     driveLimits = Drivebase.extendedLimits,
    //     name = "L2"
    // )
    
    // val L3 = SuperstructureState(
    //     armAngle = scoreAngle,
    //     driveLimits = Drivebase.extendedLimits,
    //     name = "L3"
    // )
    
    // val L4 = SuperstructureState(
    //     armAngle = Degrees.of(20.0),
    //     driveLimits = Drivebase.extendedLimits,
    //     name = "L4"
    // )
}