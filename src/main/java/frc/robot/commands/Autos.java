package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Drivebase;
import java.util.List;
import java.util.function.Supplier;
import lib.autos.AutoRoutine;
import lib.autos.AutoRoutine.AutoAction;
import lib.math.geometry.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Collection of autonomous routine factories. */
public final class Autos {
  private final Drivebase drivebase;
  private final Superstructure superstructure;

  private final LoggedDashboardChooser<Supplier<Command>> chooser =
      new LoggedDashboardChooser<>("auto");

  private final AutoRoutine ijRoutine;
  private final AutoRoutine i4Routine;
  private final AutoRoutine abcRoutine;
  private final AutoRoutine feRoutine;
  private final AutoRoutine f4Routine;
  private final AutoRoutine idleRoutine;

  public Autos(Drivebase drivebase, Superstructure superstructure) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;

    this.abcRoutine =
        new AutoRoutine(
            List.of(
                new AutoAction(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L4, true),
                new AutoAction(FieldConstants.Reef.Branch.B, FieldConstants.Reef.Level.L2, true),
                new AutoAction(FieldConstants.Reef.Branch.C, FieldConstants.Reef.Level.L3, false),
                new AutoAction(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L3, false)),
            drivebase,
            superstructure);

    this.ijRoutine =
        new AutoRoutine(
            List.of(
                new AutoAction(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
                new AutoAction(FieldConstants.Reef.Branch.J, FieldConstants.Reef.Level.L4, true)),
            drivebase,
            superstructure);

    this.i4Routine =
        new AutoRoutine(
            List.of(
                new AutoAction(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true)),
            drivebase,
            superstructure);

    this.feRoutine =
        new AutoRoutine(
            List.of(
                new AutoAction(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true),
                new AutoAction(FieldConstants.Reef.Branch.E, FieldConstants.Reef.Level.L4, true)),
            drivebase,
            superstructure);

    this.f4Routine =
        new AutoRoutine(
            List.of(
                new AutoAction(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true)),
            drivebase,
            superstructure);

    this.idleRoutine = new AutoRoutine(List.of(), drivebase, superstructure);

    chooser.addOption("IJ", ijRoutine::build);
    chooser.addOption("I4", i4Routine::build);
    chooser.addDefaultOption("DRIVE_FORWARDS", this::driveForwards);
  }

  public Command driveForwards() {
    return Commands.sequence(
        Commands.runOnce(() -> drivebase.applyChassisSpeeds(new ChassisSpeeds(-3.0, 0.0, 0.0)), drivebase),
        new WaitCommand(5.0),
        drivebase.getStopCmd());
  }

  public Command getSelectedRoutine() {
    return chooser.get().get();
  }
}
