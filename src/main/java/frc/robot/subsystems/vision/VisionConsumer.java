package frc.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

@FunctionalInterface
public interface VisionConsumer {
  void accept(Pose2d pose, double timestamp, Vector<N3> stdDevs);
}
