package lib.math;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;

/** Utilities for working with {@link Vector} and {@link Rotation2d}. */
public final class VectorFunctions {
  private VectorFunctions() {}

  public static Vector<N2> toVector(Rotation2d rotation) {
    return VecBuilder.fill(rotation.getCos(), rotation.getSin());
  }

  public static Rotation2d toRotation(Vector<N2> vector) {
    return new Rotation2d(vector.get(0, 0), vector.get(1, 0));
  }

  public static Rotation2d rotationFromVector(Vector<N2> vector) {
    return toRotation(vector);
  }

  public static Vector<N2> vectorFromRotation(Rotation2d rotation) {
    return toVector(rotation);
  }
}
