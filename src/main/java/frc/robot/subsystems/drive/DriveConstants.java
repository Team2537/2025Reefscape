package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.*;

/** Constants for drive subsystem configuration. */
public class DriveConstants {
    private static final double MAX_LINEAR_VELOCITY_MPS = 4.5;
    private static final double DRIVEBASE_RADIUS_METERS = Drive.DRIVE_BASE_RADIUS;
    private static final double MAX_ANGULAR_VELOCITY_RPS = MAX_LINEAR_VELOCITY_MPS / DRIVEBASE_RADIUS_METERS;

    public static final PathConstraints DEFAULT_LIMITS = new PathConstraints(
            MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
            MetersPerSecondPerSecond.of(14.5),
            edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
            DegreesPerSecondPerSecond.of(900.0));

    public static final PathConstraints EXTENDED_LIMITS = new PathConstraints(
            MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
            MetersPerSecondPerSecond.of(14.5),
            edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
            DegreesPerSecondPerSecond.of(900.0));

    public static final PathConstraints INTAKE_LIMITS = new PathConstraints(
            FeetPerSecond.of(3.0),
            MetersPerSecondPerSecond.of(5.5),
            edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
            DegreesPerSecondPerSecond.of(1200.0));

    public static final PathConstraints AUTO_LIMITS = new PathConstraints(
            MetersPerSecond.of(3.0),
            MetersPerSecondPerSecond.of(14.5),
            DegreesPerSecond.of(540.0),
            DegreesPerSecondPerSecond.of(720.0));

    private DriveConstants() {
    }
}
