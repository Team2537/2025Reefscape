package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.swerve.AlignmentCommand;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoals;
import frc.robot.subsystems.vision.Vision;
import frc.robot.generated.TunerConstants;
import lib.controllers.CommandButtonBoard;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class Robot extends LoggedRobot {
  public static final double UPDATE_RATE_SECONDS = 0.02;

  private static Drive drive;
  private static AlignmentState alignmentState;
  private static Vision vision;
  private static Superstructure superstructure;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandButtonBoard operatorController = new CommandButtonBoard(1, 2);
  private final CommandXboxController godController = new CommandXboxController(5);

  private Autos autos;

  public Robot() {
    HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Java, 0, WPILibVersion.Version);

    Logger.recordMetadata("Type", RobotType.TYPE.toString());
    Logger.recordMetadata("Serial Number", HALUtil.getSerialNumber());
    Logger.recordOutput("Git Dirty", BuildConstants.DIRTY == 1 ? "DIRTY" : "CLEAN");
    Logger.recordOutput("Git Branch", BuildConstants.GIT_BRANCH);
    Logger.recordOutput("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordOutput("Git Date", BuildConstants.GIT_DATE);

    switch (RobotType.MODE) {
      case REAL -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
      }
      case SIMULATION -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
      }
      case REPLAY -> {
        setUseTiming(false);
        String logFile = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logFile));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")));
      }
    }

    Logger.start();

    CanandEventLoop.getInstance();
    // FieldConstants.Reef.getNodes();

    CommandScheduler.getInstance()
        .onCommandInitialize(command -> Logger.recordOutput("commands/" + command.getName(), true));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> Logger.recordOutput("commands/" + command.getName(), false));

    CameraServer.startAutomaticCapture();

    // Initialize drive subsystem
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
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
    }

    alignmentState = new AlignmentState();
    vision = new Vision(drive::addVisionMeasurement);
    superstructure = new Superstructure(drive, alignmentState);

    // immediately set the arm to the stow position
    superstructure.getArm().setTargetAngle(SuperstructureGoals.STOW.getArmAngle());

    autos = new Autos(drive, superstructure, alignmentState);

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, driverController::getLeftY, driverController::getLeftX, () -> -driverController.getRightX()));

    driverController
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                superstructure.getForceStateCommand(() -> SuperstructureGoals.ALGAE_L2),
                superstructure.getDealgaefyL2Command(driverController.leftTrigger()::getAsBoolean)));

    driverController
        .rightTrigger()
        .onTrue(
            Commands.sequence(
                superstructure.getForceStateCommand(() -> SuperstructureGoals.ALGAE_L3),
                superstructure.getDealgaefyL3Command(driverController.rightTrigger()::getAsBoolean)));

    driverController
        .povRight()
        .onTrue(superstructure.getProcessorCommand(driverController.povRight()::getAsBoolean));

    driverController
        .povLeft()
        .onTrue(superstructure.getIntakeAlgaeCommand(driverController.povLeft()::getAsBoolean));

    driverController
        .a()
        .onTrue(superstructure.getSendToStateCommand(() -> SuperstructureGoals.STOW));

    driverController
        .x()
        .onTrue(
            Commands.sequence(
                superstructure.getForceStateCommand(() -> SuperstructureGoals.L1),
                superstructure.getScoreCommand(driverController.x()::getAsBoolean)));

    driverController
        .y()
        .onTrue(AlignmentCommand.tagRelativeAlign(drive, alignmentState, vision, 0.45, 0.0));

    driverController
        .b()
        .onTrue(superstructure.getIntakeCommand(driverController.b()::getAsBoolean));

    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
  }

  @Override
  public void robotPeriodic() {
    double armAngle = superstructure.getArm().getAngle().getRadians();
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d(0.28, 0.0, 0.275, new Rotation3d(0.0, armAngle - 1.0, 0.0)));

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autos.getSelectedRoutine().schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    MechanismVisualizer.updatePoses();
  }

  public static double getUpdateRateSec() {
    return UPDATE_RATE_SECONDS;
  }

  public static Drive getDrive() {
    return drive;
  }

  public static AlignmentState getAlignmentState() {
    return alignmentState;
  }

  public static Vision getVision() {
    return vision;
  }

  public static Superstructure getSuperstructure() {
    return superstructure;
  }
}
