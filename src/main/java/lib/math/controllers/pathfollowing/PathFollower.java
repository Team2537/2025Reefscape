package lib.math.controllers.pathfollowing;

import choreo.trajectory.SwerveSample;
import java.util.function.Consumer;

/** Functional interface describing a path follower. */
@FunctionalInterface
public interface PathFollower extends Consumer<SwerveSample> {
  @Override
  void accept(SwerveSample sample);
}
