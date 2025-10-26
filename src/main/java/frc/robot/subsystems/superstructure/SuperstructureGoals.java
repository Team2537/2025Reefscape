package frc.robot.subsystems.superstructure;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;

/** Predefined superstructure targets. */
public final class SuperstructureGoals {
    private SuperstructureGoals() {
    }

    public static final SuperstructureState STOW = new SuperstructureState(Rotation2d.fromDegrees(63.0),
            DriveConstants.DEFAULT_LIMITS, null, "Stow");

    public static final SuperstructureState PRE_ALGAE_L2 = new SuperstructureState(
            Rotation2d.fromDegrees(38.0), DriveConstants.EXTENDED_LIMITS, null, "Pre Algae L2");

    public static final SuperstructureState PRE_ALGAE_L3 = new SuperstructureState(
            Rotation2d.fromDegrees(125.0), DriveConstants.EXTENDED_LIMITS, null, "Pre Algae L3");

    public static final SuperstructureState ALGAE_L2 = new SuperstructureState(
            Rotation2d.fromDegrees(38.0), DriveConstants.EXTENDED_LIMITS, null, "Algae L2");

    public static final SuperstructureState ALGAE_L3 = new SuperstructureState(
            Rotation2d.fromDegrees(125.0), DriveConstants.EXTENDED_LIMITS, null, "Algae L3");

    public static final SuperstructureState L1 = new SuperstructureState(Rotation2d.fromDegrees(34.0),
            DriveConstants.EXTENDED_LIMITS, null, "L1");

    public static final SuperstructureState GROUND = new SuperstructureState(Rotation2d.fromDegrees(0.0),
            DriveConstants.EXTENDED_LIMITS, null, "Ground");

    public static final SuperstructureState PROCESSOR = new SuperstructureState(
            Rotation2d.fromDegrees(35.0), DriveConstants.EXTENDED_LIMITS, null, "Processor");

    public static final SuperstructureState ALGAE_INTAKE = new SuperstructureState(
            Rotation2d.fromDegrees(30.0), DriveConstants.EXTENDED_LIMITS, null, "Algae Intake");
}
