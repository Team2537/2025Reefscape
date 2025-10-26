package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

/** Tracks the current alignment state of the drive subsystem. */
public class AlignmentState {
    public enum State {
        ALIGNING,
        ALIGNED_ALGAE,
        ALIGNED_CORAL,
        ALIGNED_SOURCE,
        DRIVING
    }

    private State currentState = State.DRIVING;
    private PathConstraints currentLimits = DriveConstants.AUTO_LIMITS;

    public State getState() {
        return currentState;
    }

    public void setState(State state) {
        this.currentState = state;
    }

    public PathConstraints getLimits() {
        return currentLimits;
    }

    public void setLimits(PathConstraints limits) {
        this.currentLimits = limits;
    }
}
