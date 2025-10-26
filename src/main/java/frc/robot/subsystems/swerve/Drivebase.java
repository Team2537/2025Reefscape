package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotType;
import frc.robot.subsystems.swerve.Drivebase.AlignmentState;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.SwerveModule;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;

/** Robot swerve drive subsystem. */
public final class Drivebase extends SubsystemBase {
  private final SwerveModule[] modules = {
    new SwerveModule(1, 2, 2, false, true, Rotation2d.fromRotations(0.261), MODULE_TRANSLATIONS[0]),
    new SwerveModule(3, 4, 4, false, true, Rotation2d.fromRotations(-0.4367), MODULE_TRANSLATIONS[1]),
    new SwerveModule(5, 6, 6, false, true, Rotation2d.fromRotations(0.1186), MODULE_TRANSLATIONS[2]),
    new SwerveModule(7, 8, 8, false, true, Rotation2d.fromRotations(0.362), MODULE_TRANSLATIONS[3])
  };

  private final GyroIO gyro =
      switch (RobotType.MODE) {
        case SIMULATION -> new GyroIOSim(this::getChassisSpeeds);
        case REAL -> new GyroIOPigeon2(9);
        case REPLAY -> new GyroIO() {};
      };

  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  private final SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kinematics,
          gyroInputs.yaw,
          getWheelPositionsArray(),
          new Pose2d(),
          VecBuilder.fill(0.1, 0.1, 0.1),
          VecBuilder.fill(0.9, 0.9, 0.9));

  private final SysIdRoutine driveSysIdVolts =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              Seconds.of(2.0),
              state -> Logger.recordOutput(getName() + "/sysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> Arrays.stream(modules).forEach(module -> module.characterizeDriveVoltage(volts)),
              null,
              this));

  private final SysIdRoutine steerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              state -> Logger.recordOutput(getName() + "/state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> Arrays.stream(modules).forEach(module -> module.characterizeSteerVoltage(volts)),
              null,
              this));

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint lastSetpoint;

  private final PIDController headingPid = new PIDController(5.0, 0.0, 0.0);

  private PathConstraints limits = AUTO_LIMITS;
  private final RobotConfig robotConfig;

  private boolean hasAppliedOperatorPerspective = false;
  private Rotation2d operatorPerspective = BLUE_PERSPECTIVE;
  private boolean slowModeEnabled = false;
  private AlignmentState alignmentState = AlignmentState.DRIVING;

  public Drivebase() {
    super("drivebase");
    headingPid.enableContinuousInput(0.0, 1.0);

    robotConfig = loadRobotConfig();
    setpointGenerator = new SwerveSetpointGenerator(robotConfig, RPM.of(560.0));
    lastSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(), getWheelStatesArray(), DriveFeedforwards.zeros(modules.length));

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, _) -> applyChassisSpeeds(speeds),
        new PPHolonomicDriveController(new PIDConstants(7.0), new PIDConstants(5.0)),
        robotConfig,
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("auto/activePath", path.toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("auto/targetPose", Pose2d.struct, pose));
  }

  private RobotConfig loadRobotConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (Exception ex) {
      ex.printStackTrace();
      return null;
    }
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
    lastSetpoint =
        setpointGenerator.generateSetpoint(
            lastSetpoint,
            speeds,
            RobotBase.isReal() ? limits : DEFAULT_LIMITS,
            Robot.getUpdateRateSec());

    for (int i = 0; i < modules.length; i++) {
      modules[i].applyState(lastSetpoint.moduleStates()[i], moduleForces.get(i));
    }
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds, PathConstraints constraints) {
    lastSetpoint =
        setpointGenerator.generateSetpoint(
            lastSetpoint, speeds, constraints, Robot.getUpdateRateSec());
    for (int i = 0; i < modules.length; i++) {
      modules[i].applyState(lastSetpoint.moduleStates()[i]);
    }
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds) {
    applyChassisSpeeds(speeds, Robot.isTeleop() ? limits : DEFAULT_LIMITS);
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path).andThen(getStopCmd());
  }

  public void runWheelRadiusCharacterization(double omegaRadPerSec) {
    applyChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omegaRadPerSec));
  }

  public Command getStopCmd() {
    return Commands.runOnce(
        () -> {
          for (int i = 0; i < modules.length; i++) {
            SwerveModuleState state = getWheelStatesArray()[i];
            modules[i].applyState(new SwerveModuleState(0.0, state.angle));
          }
        });
  }

  public Command getForceStateCommand(Supplier<AlignmentState> newState) {
    return Commands.runOnce(() -> alignmentState = newState.get());
  }

  public void toggleSlowMode() {
    slowModeEnabled = !slowModeEnabled;
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
    return run(() -> {
          double forwardInput = forward.getAsDouble();
          double strafeInput = strafe.getAsDouble();
          double rotationInput = rotation.getAsDouble();

          double magnitude = Math.hypot(strafeInput, forwardInput);
          magnitude = Math.pow(magnitude, exponent);
          Rotation2d direction = Rotation2d.fromRadians(Math.atan2(forwardInput, strafeInput));

          double rotationSpeed = rotationInput;
          Rotation2d targetHeading = headingTarget.get();
          if (targetHeading != null && Math.abs(rotationInput) < 0.1) {
            Rotation2d goal =
                AutoBuilder.shouldFlip() ? targetHeading.plus(Rotation2d.k180deg) : targetHeading;
            rotationSpeed = headingPid.calculate(getPose().getRotation().getRotations(), goal.getRotations());
          }

          double forwardScaled = magnitude * direction.getSin();
          double strafeScaled = magnitude * direction.getCos();

          if (alignmentState != AlignmentState.DRIVING && magnitude > inchesToMeters(2.0)) {
            alignmentState = AlignmentState.DRIVING;
          }

          double linearScale = selectScale(shouldBoostSupplier, shouldSlowSupplier);
          double angularScale = selectScale(shouldBoostSupplier, shouldSlowSupplier);

          ChassisSpeeds speeds;
          if (shouldFieldOrient.getAsBoolean()) {
            speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardScaled * MAX_LINEAR_VELOCITY_MPS * linearScale,
                    strafeScaled * MAX_LINEAR_VELOCITY_MPS * linearScale,
                    rotationSpeed * MAX_ANGULAR_VELOCITY_RPS * angularScale,
                    getPose().getRotation().plus(operatorPerspective));
          } else {
            speeds =
                new ChassisSpeeds(
                    forwardScaled * MAX_LINEAR_VELOCITY_MPS * linearScale,
                    strafeScaled * MAX_LINEAR_VELOCITY_MPS * linearScale,
                    rotationSpeed * MAX_ANGULAR_VELOCITY_RPS * angularScale);
          }

          PathConstraints selected =
              shouldSlowSupplier.getAsBoolean()
                  ? SLOWMODE_LIMITS
                  : (shouldBoostSupplier.getAsBoolean() ? DEFAULT_LIMITS : limits);

          applyChassisSpeeds(speeds, selected);
        });
  }

  private double selectScale(BooleanSupplier boost, BooleanSupplier slow) {
    if (slow.getAsBoolean()) {
      return 0.5;
    }
    return boost.getAsBoolean() ? 1.0 : 0.75;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public Command resetHeading() {
    return Commands.runOnce(() -> gyro.setYaw(new Rotation2d()));
  }

  public Command driveSysId() {
    return Commands.runOnce(() -> Arrays.stream(modules).forEach(module -> module.applyState(new SwerveModuleState())))
        .withTimeout(3.0)
        .andThen(
            steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1.0),
            steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(1.0),
            steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1.0),
            steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stdDevs) {
    if (RobotBase.isReal()) {
      odometry.addVisionMeasurement(pose, timestamp, stdDevs);
    }
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroInputs);
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
      Logger.processInputs(getName() + "/modules/" + i, modules[i].getInputs());
    }
    Logger.processInputs(getName() + "/gyro", gyroInputs);

    odometry.update(gyroInputs.yaw, getWheelPositionsArray());

    Logger.recordOutput(getName() + "/pose", Pose2d.struct, getPose());
    Logger.recordOutput(getName() + "/wheelStates", getWheelStatesArray());
    Logger.recordOutput(getName() + "/desiredStates", getDesiredStatesArray());
    Logger.recordOutput(getName() + "/wheelPositions", getWheelPositionsArray());
    Logger.recordOutput(getName() + "/moduleForces", getModuleForceArray());
    Logger.recordOutput(getName() + "/limits", limits);
    Logger.recordOutput(getName() + "/operatorPerspective", Rotation2d.struct, operatorPerspective);
    Logger.recordOutput(getName() + "/slowModeEnabled", slowModeEnabled);
    Logger.recordOutput(getName() + "/alignmentState", alignmentState);
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelStatesArray());
  }

  public SwerveModuleState[] getWheelStatesArray() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  public SwerveModuleState[] getDesiredStatesArray() {
    return Arrays.stream(modules).map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
  }

  public SwerveModulePosition[] getWheelPositionsArray() {
    return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getModuleForceArray() {
    return Arrays.stream(modules).map(SwerveModule::getWheelForce).toArray(SwerveModuleState[]::new);
  }

  public double[] getWheelRadiusCharacterizationAngles() {
    return Arrays.stream(modules).mapToDouble(SwerveModule::getRadiusCharacterizationAngleRad).toArray();
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

  private static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(inchesToMeters(12.875), inchesToMeters(11.875)),
        new Translation2d(inchesToMeters(12.875), inchesToMeters(-11.875)),
        new Translation2d(inchesToMeters(-12.875), inchesToMeters(11.875)),
        new Translation2d(inchesToMeters(-12.875), inchesToMeters(-11.875))
      };

  private static final double MAX_LINEAR_VELOCITY_MPS = 4.5;
  private static final double DRIVEBASE_RADIUS_METERS =
      Arrays.stream(MODULE_TRANSLATIONS).mapToDouble(Translation2d::getNorm).max().orElse(0.0);
  private static final double MAX_ANGULAR_VELOCITY_RPS =
      MAX_LINEAR_VELOCITY_MPS / DRIVEBASE_RADIUS_METERS;

  private static final PathConstraints DEFAULT_LIMITS =
      new PathConstraints(
          MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
          MetersPerSecondPerSecond.of(14.5),
          RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
          DegreesPerSecondPerSecond.of(1500.0));

  private static final PathConstraints EXTENDED_LIMITS =
      new PathConstraints(
          MetersPerSecond.of(MAX_LINEAR_VELOCITY_MPS),
          MetersPerSecondPerSecond.of(14.5),
          RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
          DegreesPerSecondPerSecond.of(1500.0));

  private static final PathConstraints SLOWMODE_LIMITS =
      new PathConstraints(
          FeetPerSecond.of(3.0),
          MetersPerSecondPerSecond.of(5.5),
          DegreesPerSecond.of(180.0),
          DegreesPerSecondPerSecond.of(1500.0));

  private static final PathConstraints INTAKE_LIMITS =
      new PathConstraints(
          FeetPerSecond.of(3.0),
          MetersPerSecondPerSecond.of(5.5),
          RadiansPerSecond.of(MAX_ANGULAR_VELOCITY_RPS),
          DegreesPerSecondPerSecond.of(2500.0));

  private static final PathConstraints AUTO_LIMITS =
      new PathConstraints(
          MetersPerSecond.of(3.0),
          MetersPerSecondPerSecond.of(14.5),
          DegreesPerSecond.of(540.0),
          DegreesPerSecondPerSecond.of(720.0));

  private static final Rotation2d BLUE_PERSPECTIVE = Rotation2d.fromDegrees(0.0);
  private static final Rotation2d RED_PERSPECTIVE = Rotation2d.fromDegrees(180.0);

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
