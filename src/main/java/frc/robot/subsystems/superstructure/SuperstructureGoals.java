package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.swerve.Drivebase;
import lib.math.units.into;
import java.nio.ByteBuffer;
import java.util.Optional;

// TODO: these need to be tuned significantly
public class SuperstructureGoals {
    public static final SuperstructureState STOW = new SuperstructureState(
        Degrees.of(63.0),
        Drivebase.defaultLimits,
        "Stow"
    );

    // public static final SuperstructureState ALGAE_STOW = new SuperstructureState(
    //     Degrees.of(-20.0),
    //     Drivebase.defaultLimits,
    //     "Algae Stow"
    // );

    public static final SuperstructureState PRE_ALGAE_L2 = new SuperstructureState(
        Degrees.of(38.0),
        Drivebase.extendedLimits,
        "Pre Algae L2"
    );

    public static final SuperstructureState PRE_ALGAE_L3 = new SuperstructureState(
        Degrees.of(125.0),
        Drivebase.extendedLimits,
        "Pre Algae L3"
    );

    public static final SuperstructureState ALGAE_L2 = new SuperstructureState(
        Degrees.of(38.0),
        Drivebase.extendedLimits,
        "Algae L2"
    );

    public static final SuperstructureState ALGAE_L3 = new SuperstructureState(
        Degrees.of(125.0),
        Drivebase.extendedLimits,
        "Algae L3"
    );

    // public static final SuperstructureState BARGE = new SuperstructureState(
    //     Degrees.of(-20.0),
    //     Drivebase.extendedLimits,
    //     "Barge"
    // );
    
    public static final SuperstructureState L1 = new SuperstructureState(
        Degrees.of(34.0), // needs tuning
        Drivebase.extendedLimits,
        "L1"
    );

    public static final SuperstructureState GROUND = new SuperstructureState(
        Degrees.of(0.0),
        Drivebase.extendedLimits,
        "Ground"
    );

    public static final SuperstructureState PROCESSOR = new SuperstructureState(
        Degrees.of(35.0),
        Drivebase.extendedLimits,
        "Processor"
    );

    public static final SuperstructureState ALGAE_INTAKE = new SuperstructureState(
        Degrees.of(30.0),
        Drivebase.extendedLimits,
        "Algae Intake"
    );
    
    // public static final SuperstructureState L2 = new SuperstructureState(
    //     scoreAngle,
    //     Drivebase.extendedLimits,
    //     "L2"
    // );
    
    // public static final SuperstructureState L3 = new SuperstructureState(
    //     scoreAngle,
    //     Drivebase.extendedLimits,
    //     "L3"
    // );
    
    // public static final SuperstructureState L4 = new SuperstructureState(
    //     Degrees.of(20.0),
    //     Drivebase.extendedLimits,
    //     "L4"
    // );
}
