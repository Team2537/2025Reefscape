package lib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Objects;
import java.util.Optional;

/** Stores blue and red variants of a geometry object with automatic alliance selection. */
public final class Flippable2d<T> {
  private final T blue;
  private final T red;

  private Flippable2d(T blue, T red) {
    this.blue = blue;
    this.red = red;
  }

  public static Flippable2d<Translation2d> withBlue(Translation2d blue, Flipper2d flipper) {
    return new Flippable2d<>(blue, flipper.flipTranslation(blue));
  }

  public static Flippable2d<Translation2d> withRed(Translation2d red, Flipper2d flipper) {
    return new Flippable2d<>(flipper.flipTranslation(red), red);
  }

  public static Flippable2d<Rotation2d> withBlue(Rotation2d blue, Flipper2d flipper) {
    return new Flippable2d<>(blue, flipper.flipRotation(blue));
  }

  public static Flippable2d<Rotation2d> withRed(Rotation2d red, Flipper2d flipper) {
    return new Flippable2d<>(flipper.flipRotation(red), red);
  }

  public static Flippable2d<Pose2d> withBlue(Pose2d blue, Flipper2d flipper) {
    return new Flippable2d<>(blue, flipper.flipPose(blue));
  }

  public static Flippable2d<Pose2d> withRed(Pose2d red, Flipper2d flipper) {
    return new Flippable2d<>(flipper.flipPose(red), red);
  }

  public static Flippable2d<Transform2d> withBlue(Transform2d blue, Flipper2d flipper) {
    return new Flippable2d<>(blue, flipper.flipTransform(blue));
  }

  public static Flippable2d<Transform2d> withRed(Transform2d red, Flipper2d flipper) {
    return new Flippable2d<>(flipper.flipTransform(red), red);
  }

  public static Flippable2d<Twist2d> withBlue(Twist2d blue, Flipper2d flipper) {
    return new Flippable2d<>(blue, flipper.flipTwist(blue));
  }

  public static Flippable2d<Twist2d> withRed(Twist2d red, Flipper2d flipper) {
    return new Flippable2d<>(flipper.flipTwist(red), red);
  }

  public T getBlue() {
    return blue;
  }

  public T getRed() {
    return red;
  }

  public T getOurs() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? red : blue;
  }

  public T getTheirs() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? blue : red;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!(obj instanceof Flippable2d<?> other)) {
      return false;
    }
    return Objects.equals(blue, other.blue) && Objects.equals(red, other.red);
  }

  @Override
  public int hashCode() {
    return Objects.hash(blue, red);
  }

  @Override
  public String toString() {
    return "Flippable2d{" + "blue=" + blue + ", red=" + red + '}';
  }
}
