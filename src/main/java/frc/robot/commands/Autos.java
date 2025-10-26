package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.List;
import java.util.function.Supplier;
import lib.autos.AutoRoutine;
import lib.autos.AutoRoutine.AutoAction;
import lib.math.geometry.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Collection of autonomous routine factories. */
public final class Autos {
    private final Drive drive;
    private final Superstructure superstructure;
    private final AlignmentState alignmentState;

    private final LoggedDashboardChooser<Supplier<Command>> chooser = new LoggedDashboardChooser<>("auto");

    private final AutoRoutine ijRoutine;
    private final AutoRoutine i4Routine;
    private final AutoRoutine abcRoutine;
    private final AutoRoutine feRoutine;
    private final AutoRoutine f4Routine;
    private final AutoRoutine idleRoutine;

    public Autos(Drive drive, Superstructure superstructure, AlignmentState alignmentState) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.alignmentState = alignmentState;

        this.abcRoutine = new AutoRoutine(
                List.of(
                        new AutoAction(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L4, true),
                        new AutoAction(FieldConstants.Reef.Branch.B, FieldConstants.Reef.Level.L2, true),
                        new AutoAction(FieldConstants.Reef.Branch.C, FieldConstants.Reef.Level.L3, false),
                        new AutoAction(FieldConstants.Reef.Branch.A, FieldConstants.Reef.Level.L3, false)),
                drive,
                superstructure);

        this.ijRoutine = new AutoRoutine(
                List.of(
                        new AutoAction(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true),
                        new AutoAction(FieldConstants.Reef.Branch.J, FieldConstants.Reef.Level.L4, true)),
                drive,
                superstructure);

        this.i4Routine = new AutoRoutine(
                List.of(
                        new AutoAction(FieldConstants.Reef.Branch.I, FieldConstants.Reef.Level.L4, true)),
                drive,
                superstructure);

        this.feRoutine = new AutoRoutine(
                List.of(
                        new AutoAction(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true),
                        new AutoAction(FieldConstants.Reef.Branch.E, FieldConstants.Reef.Level.L4, true)),
                drive,
                superstructure);

        this.f4Routine = new AutoRoutine(
                List.of(
                        new AutoAction(FieldConstants.Reef.Branch.F, FieldConstants.Reef.Level.L4, true)),
                drive,
                superstructure);

        this.idleRoutine = new AutoRoutine(List.of(), drive, superstructure);

        chooser.addOption("IJ", ijRoutine::build);
        chooser.addOption("I4", i4Routine::build);
        chooser.addDefaultOption("DRIVE_FORWARDS", this::driveForwards);
    }

    public Command driveForwards() {
        return Commands.sequence(
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-3.0, 0.0, 0.0)), drive),
                new WaitCommand(5.0),
                Commands.runOnce(drive::stopWithX, drive));
    }

    public Command getSelectedRoutine() {
        return chooser.get().get();
    }
}
