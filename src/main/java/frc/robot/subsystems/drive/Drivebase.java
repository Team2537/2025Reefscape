package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotType;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;

/** Adapter around AdvantageKit Drive to preserve existing project APIs. */
public final class Drivebase extends SubsystemBase {
  private final Drive drive;

  private PathConstraints limits = AUTO_LIMITS;
  private AlignmentState alignmentState = AlignmentState.DRIVING;

  public Drivebase() {
    super("drivebase");
    switch (RobotType.MODE) {
      case REAL ->
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIMULATION ->
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
      default ->
        drive = new Drive(
            new GyroIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            });
    }
  }

  public Command getDriveCommand(
      DoubleSupplier forward,
      DoubleSupplier strafe,
      DoubleSupplier rotation,
      BooleanSupplier shouldFieldOrient,
      BooleanSupplier shouldBoostSupplier,
      BooleanSupplier shouldSlowSupplier,
      Supplier<Rotation2d> headingTarget,
      int exponent) {
    // Create a command that requires this subsystem and forwards to the new drive
    return run(
        () -> {
          // Read inputs (already deadbanded in Robot)
          double xIn = forward.getAsDouble();
          double yIn = strafe.getAsDouble();
          double omegaIn = rotation.getAsDouble();

          // Square for precision while preserving sign
          double xCmd = Math.copySign(xIn * xIn, xIn);
          double yCmd = Math.copySign(yIn * yIn, yIn);
          double omegaCmd = Math.copySign(omegaIn * omegaIn, omegaIn);

          // Optional scaling for boost/slow
          double scale = shouldSlowSupplier.getAsBoolean() ? 0.5 : (shouldBoostSupplier.getAsBoolean() ? 1.0 : 0.75);

          double vx = xCmd * drive.getMaxLinearSpeedMetersPerSec() * scale;
          double vy = yCmd * drive.getMaxLinearSpeedMetersPerSec() * scale;
          double omega = omegaCmd * drive.getMaxAngularSpeedRadPerSec() * scale;

          if (shouldFieldOrient.getAsBoolean()) {
            boolean isFlipped = edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                && edu.wpi.first.wpilibj.DriverStation.getAlliance()
                    .get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
            Rotation2d ref = isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, ref));
          } else {
            drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
          }
        });
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds) {
    drive.runVelocity(speeds);
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds, PathConstraints ignored) {
    drive.runVelocity(speeds);
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path).andThen(Commands.runOnce(drive::stopWithX, drive));
  }

  public void runWheelRadiusCharacterization(double omegaRadPerSec) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omegaRadPerSec));
  }

  public Command getStopCmd() {
    return Commands.runOnce(drive::stopWithX, drive);
  }

  public Command getForceStateCommand(Supplier<AlignmentState> newState) {
    return Commands.runOnce(() -> alignmentState = newState.get());
  }

  public Command resetHeading() {
    return Commands.runOnce(
        () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)));
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stdDevs) {
    @SuppressWarnings("unchecked")
    Matrix<N3, N1> stdDevsMatrix = (Matrix<N3, N1>) (Object) stdDevs;
    drive.addVisionMeasurement(pose, timestamp, stdDevsMatrix);
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public double[] getWheelRadiusCharacterizationAngles() {
    return drive.getWheelRadiusCharacterizationPositions();
  }

  public static double getDrivebaseRadiusMeters() {
    return Drive.DRIVE_BASE_RADIUS;
  }

  public AlignmentState getAlignmentState() {
    return alignmentState;
  }

  public void setAlignmentState(AlignmentState state) {
    alignmentState = state;
  }

  public void setLimits(PathConstraints newLimits) {
    limits = newLimits;
  }

  public PathConstraints getLimits() {
    return limits;
  }

  public enum AlignmentState {
    ALIGNING,
    ALIGNED_ALGAE,
    ALIGNED_CORAL,
    ALIGNED_SOURCE,
    DRIVING
  }

  private static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
      new Translation2d(inchesToMeters(12.875), inchesToMeters(11.875)),
      new Translation2d(inchesToMeters(12.875), inchesToMeters(-11.875)),
      new Translation2d(inchesToMeters(-12.875), inchesToMeters(11.875)),
      new Translation2d(inchesToMeters(-12.875), inchesToMeters(-11.875))
  };

  private static final double MAX_LINEAR_VELOCITY_MPS = 4.5;
  private static final double DRIVEBASE_RADIUS_METERS = Drive.DRIVE_BASE_RADIUS;
  private static final double MAX_ANGULAR_VELOCITY_RPS = MAX_LINEAR_VELOCITY_MPS / DRIVEBASE_RADIUS_METERS;

  private static final PathConstraints DEFAULT_LIMITS = new PathConstraints(
      MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
      MetersPerSecondPerSecond.of(14.5),
      edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
      DegreesPerSecondPerSecond.of(900.0));

  private static final PathConstraints EXTENDED_LIMITS = new PathConstraints(
      MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
      MetersPerSecondPerSecond.of(14.5),
      edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
      DegreesPerSecondPerSecond.of(900.0));

  private static final PathConstraints SLOWMODE_LIMITS = new PathConstraints(
      FeetPerSecond.of(3.0),
      MetersPerSecondPerSecond.of(5.5),
      DegreesPerSecond.of(180.0),
      DegreesPerSecondPerSecond.of(720.0));

  private static final PathConstraints INTAKE_LIMITS = new PathConstraints(
      FeetPerSecond.of(3.0),
      MetersPerSecondPerSecond.of(5.5),
      edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
      DegreesPerSecondPerSecond.of(1200.0));

  private static final PathConstraints AUTO_LIMITS = new PathConstraints(
      MetersPerSecond.of(3.0),
      MetersPerSecondPerSecond.of(14.5),
      DegreesPerSecond.of(540.0),
      DegreesPerSecondPerSecond.of(720.0));

  public static PathConstraints getDefaultLimits() {
    return DEFAULT_LIMITS;
  }

  public static PathConstraints getExtendedLimits() {
    return EXTENDED_LIMITS;
  }

  public static PathConstraints getIntakeLimits() {
    return INTAKE_LIMITS;
  }
}
