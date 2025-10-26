package lib.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoals;
import frc.robot.subsystems.swerve.Drivebase;
import java.util.List;

/** Describes an autonomous routine built from branch actions. */
public final class AutoRoutine {
  public record AutoAction(
      lib.math.geometry.FieldConstants.Reef.Branch branch,
      lib.math.geometry.FieldConstants.Reef.Level level,
      boolean top) {}

  private final List<AutoAction> actions;
  private final Drivebase drivebase;
  private final Superstructure superstructure;

  public AutoRoutine(List<AutoAction> actions, Drivebase drivebase, Superstructure superstructure) {
    this.actions = actions;
    this.drivebase = drivebase;
    this.superstructure = superstructure;
  }

  public Command build() {
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    PathPlannerPath startPath = getPathFromStart(actions.get(0).branch());

    sequence.addCommands(
        Commands.sequence(
            AutoBuilder.resetOdom(startPath.getStartingHolonomicPose().orElseGet(Pose2d::new)),
            superstructure.getSendToStateCommand(() -> SuperstructureGoals.STOW),
            drivebase.followPath(startPath)));

    for (int index = 0; index < actions.size(); index++) {
      AutoAction action = actions.get(index);

      sequence.addCommands(
          Commands.parallel(
              drivebase.followPath(getPathToBranch(action.branch(), action.top())).onlyIf(() -> index != 0),
              switch (action.level()) {
                case L1 -> superstructure.getForceStateCommand(() -> SuperstructureGoals.L1);
                case L2 -> superstructure.getForceStateCommand(() -> SuperstructureGoals.STOW);
                case L3 -> superstructure.getForceStateCommand(() -> SuperstructureGoals.STOW);
                case L4 -> superstructure.getForceStateCommand(() -> SuperstructureGoals.STOW);
              }),
          drivebase.getStopCmd(),
          Commands.waitSeconds(0.75),
          superstructure.getScoreCommand(() -> true));

      if (index != actions.size() - 1) {
        sequence.addCommands(
            Commands.parallel(
                drivebase.followPath(getPathToSource(action.branch())),
                superstructure.getSendToStateCommand(() -> SuperstructureGoals.STOW)),
            drivebase.getStopCmd(),
            Commands.waitSeconds(2.0));
      }
    }

    return sequence;
  }

  private PathPlannerPath getPathToBranch(
      lib.math.geometry.FieldConstants.Reef.Branch branch, boolean top) {
    if (top) {
      return PathPlannerPath.fromPathFile("ts_to_" + branch.name());
    }
    return PathPlannerPath.fromPathFile("bs_to_" + branch.name());
  }

  private PathPlannerPath getPathToSource(lib.math.geometry.FieldConstants.Reef.Branch startBranch) {
    boolean topSource =
        List.of(
                lib.math.geometry.FieldConstants.Reef.Branch.A,
                lib.math.geometry.FieldConstants.Reef.Branch.L,
                lib.math.geometry.FieldConstants.Reef.Branch.K,
                lib.math.geometry.FieldConstants.Reef.Branch.J,
                lib.math.geometry.FieldConstants.Reef.Branch.I,
                lib.math.geometry.FieldConstants.Reef.Branch.H)
            .contains(startBranch);

    return PathPlannerPath.fromPathFile(
        startBranch.name() + "_to_" + (topSource ? "ts" : "bs"));
  }

  private PathPlannerPath getPathFromStart(lib.math.geometry.FieldConstants.Reef.Branch branch) {
    List<lib.math.geometry.FieldConstants.Reef.Branch> topBranches =
        List.of(
            lib.math.geometry.FieldConstants.Reef.Branch.J,
            lib.math.geometry.FieldConstants.Reef.Branch.I);

    if (topBranches.contains(branch)) {
      return PathPlannerPath.fromPathFile("tstart_to_" + branch.name());
    }
    return PathPlannerPath.fromPathFile("bstart_to_" + branch.name());
  }
}
