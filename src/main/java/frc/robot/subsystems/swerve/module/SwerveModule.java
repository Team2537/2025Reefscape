package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.RobotType;
import java.util.function.Supplier;
import lib.math.VectorFunctions;
import lib.math.controllers.gains.ControllerGains;
import lib.math.controllers.gains.FeedforwardGains;
import lib.math.controllers.gains.PIDGains;
import org.littletonrobotics.junction.Logger;

/** Represents a single swerve module. */
public final class SwerveModule {
  private final int index;
  private final Translation2d modulePosition;
  private final ModuleIO io;
  private final ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
  private final Vector<N2> positiveRotationVector;

  private SwerveModuleState desiredState = new SwerveModuleState();
  private SwerveModuleState wheelForce = new SwerveModuleState();

  private static final double WHEEL_RADIUS_METERS = 0.049;
  private static final double TURN_GEARING = 150.0 / 7.0;
  private static final double DRIVE_GEARING = 6.75;

  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_MOTOR = DCMotor.getNEO(1);

  private static final ControllerGains DRIVE_TORQUE_GAINS =
      new ControllerGains(new PIDGains(10.0, 0.0, 0.0), new FeedforwardGains(1.15, 0.0, 0.0));

  private static final ControllerGains DRIVE_VELOCITY_VOLTAGE_GAINS =
      new ControllerGains(
          new PIDGains(0.88842, 0.0, 0.0), new FeedforwardGains(0.34113, 1.9168, 0.081352));

  public SwerveModule(
      int driveID,
      int turnID,
      int encoderID,
      boolean invertDrive,
      boolean invertTurn,
      Rotation2d encoderOffset,
      Translation2d modulePosition) {
    this.index = (driveID - 1) / 3;
    this.modulePosition = modulePosition;

    positiveRotationVector =
        VectorFunctions.vectorFromRotation(modulePosition.getAngle().plus(Rotation2d.fromDegrees(90.0)));

    io = createIO(driveID, turnID, encoderID, invertDrive, invertTurn, encoderOffset);
  }

  private ModuleIO createIO(
      int driveID,
      int turnID,
      int encoderID,
      boolean invertDrive,
      boolean invertTurn,
      Rotation2d encoderOffset) {
    return switch (RobotType.MODE) {
      case SIMULATION ->
          new ModuleIOSim(
              new FeedforwardGains(0.0, 12.0 / 97.0, 0.0),
              new PIDGains(0.01, 0.0, 0.0),
              DRIVE_MOTOR,
              DRIVE_GEARING,
              new FeedforwardGains(),
              new PIDGains(),
              TURN_MOTOR,
              TURN_GEARING,
              WHEEL_RADIUS_METERS);
      case REAL ->
          new ModuleIOHybridFXS(
              driveID,
              DRIVE_GEARING,
              invertDrive,
              DRIVE_TORQUE_GAINS.getFeedforward(),
              DRIVE_TORQUE_GAINS.getPid(),
              turnID,
              TURN_GEARING,
              invertTurn,
              new FeedforwardGains(0.14403, 0.0, 0.0),
              new PIDGains(50.0, 0.0, 0.5),
              encoderID,
              encoderOffset,
              Units.Meters.of(WHEEL_RADIUS_METERS));
      case REPLAY -> new ModuleIO() {};
    };
  }

  public void periodic() {
    io.updateInputs(inputs);

    double driveError = inputs.driveVelocityMetersPerSec - desiredState.speedMetersPerSecond;
    Logger.recordOutput("modules/" + index + "/driveError", driveError);

    double turnError =
        desiredState.angle.minus(inputs.absoluteTurnPosition).getRadians();
    Logger.recordOutput("modules/" + index + "/turnError", turnError);
  }

  public void applyState(SwerveModuleState targetState) {
    SwerveModuleState optimized = SwerveModuleState.optimize(targetState, inputs.absoluteTurnPosition);
    SwerveModuleState scaled = SwerveModuleState.cosineScale(optimized, inputs.absoluteTurnPosition);

    Logger.recordOutput("modules/" + index + "/preOptimize", SwerveModuleState.struct, targetState);
    Logger.recordOutput("modules/" + index + "/postOptimize", SwerveModuleState.struct, scaled);

    io.setTurnPosition(scaled.angle);
    io.setDriveVelocity(scaled.speedMetersPerSecond);
    desiredState = scaled;
    wheelForce = new SwerveModuleState(0.0, scaled.angle);
  }

  public void applyState(SwerveModuleState targetState, Vector<N2> moduleForce) {
    SwerveModuleState optimized = SwerveModuleState.optimize(targetState, inputs.absoluteTurnPosition);
    SwerveModuleState scaled = SwerveModuleState.cosineScale(optimized, inputs.absoluteTurnPosition);

    Vector<N2> wheelDirection =
        VectorFunctions.vectorFromRotation(getState().angle);
    double wheelTorqueNm = moduleForce.dot(wheelDirection) * WHEEL_RADIUS_METERS;

    io.setTurnPosition(scaled.angle);
    io.setDriveVelocity(scaled.speedMetersPerSecond, wheelTorqueNm * DRIVE_MOTOR.KtNMPerAmp);

    wheelForce = new SwerveModuleState(wheelTorqueNm, scaled.angle);
    desiredState = scaled;
  }

  public void characterizeDriveVoltage(double volts) {
    io.setTurnPosition(Rotation2d.kZero);
    io.setDriveVoltage(volts);
  }

  public void characterizeSteerVoltage(double volts) {
    io.setTurnVoltage(volts);
    io.setDriveVoltage(0.0);
  }

  public void characterizeCurrent(double currentAmps) {
    io.setTurnPosition(Rotation2d.kZero);
    io.setDriveCurrent(currentAmps);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.absoluteTurnPosition);
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.absoluteTurnPosition);
  }

  public double getRadiusCharacterizationAngleRad() {
    return inputs.drivePositionRad;
  }

  public SwerveModuleState getWheelForce() {
    return wheelForce;
  }

  public Vector<N2> getPositiveRotationVector() {
    return positiveRotationVector;
  }

  public void stop() {
    io.stop();
  }

  public int getIndex() {
    return index;
  }

  public ModuleIO.ModuleIOInputs getInputs() {
    return inputs;
  }
}
