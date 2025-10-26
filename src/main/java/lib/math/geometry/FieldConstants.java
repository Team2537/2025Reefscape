package lib.math.geometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Field-specific constants translated from Kotlin. */
public final class FieldConstants {
  private FieldConstants() {}

  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters((26.0 * 12.0) + 5.0);
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters((57.0 * 12.0) + 6.0 + 7.0 / 8.0);

  public static final Pose2d FIELD_CENTER =
      new Pose2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0, new Rotation2d());

  public static final AprilTagFieldLayout TAG_LAYOUT = loadLayout();

  public static final List<Integer> BLUE_TAG_IDS = List.of(12, 13, 16, 17, 18, 19, 20, 21, 22);
  public static final List<Integer> RED_TAG_IDS = List.of(1, 2, 3, 6, 7, 8, 9, 10, 11);

  static {
    Reef.initLogging();
    Processor.initLogging();
    Barge.initLogging();
    Source.initLogging();
  }

  private static AprilTagFieldLayout loadLayout() {
    try {
      Path layoutPath =
          Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape-andymark-nobarge.json");
      return new AprilTagFieldLayout(layoutPath);
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to load AprilTag layout", ex);
    }
  }

  public static final class Reef {
    private Reef() {}

    public static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.489325, 4.025877);

    public static final Ellipse2d REEF_ZONE =
        new Ellipse2d(
            new Pose2d(REEF_CENTER_BLUE, new Rotation2d()),
            Units.inchesToMeters(32.745545 + 20.0),
            Units.inchesToMeters(32.745545 + 20.0));

    public enum Side {
      LEFT,
      RIGHT,
      CENTER
    }

    public enum Branch {
      A,
      B,
      C,
      D,
      E,
      F,
      G,
      H,
      I,
      J,
      K,
      L
    }

    public static final double SIDE_OFFSET_METERS = Units.inchesToMeters(12.937968);

    public static final List<Pose2d> FLOOR_ALIGNMENT_POSES = createFloorAlignmentPoses();

    public static final List<List<Pose2d>> BETTER_FLOOR_POSES = createBetterFloorPoses();

    public enum Level {
      L1(0.46, 35.0, Units.inchesToMeters(30.679296)),
      L2(Units.inchesToMeters(31.218618), 35.0, Units.inchesToMeters(30.679296)),
      L3(Units.inchesToMeters(47.088688), 35.0, Units.inchesToMeters(30.679296)),
      L4(Units.inchesToMeters(71.994600), 90.0, Units.inchesToMeters(30.738196)),
      FLOOR(0.0, 0.0, Units.inchesToMeters(30.738196 + 23.25));

      private final double heightMeters;
      private final double angleDegrees;
      private final double radiusMeters;

      Level(double heightMeters, double angleDegrees, double radiusMeters) {
        this.heightMeters = heightMeters;
        this.angleDegrees = angleDegrees;
        this.radiusMeters = radiusMeters;
      }

      public double getHeightMeters() {
        return heightMeters;
      }

      public double getAngleDegrees() {
        return angleDegrees;
      }

      public double getRadiusMeters() {
        return radiusMeters;
      }
    }

    public enum ReefFace {
      S1,
      S2,
      S3,
      S4,
      S5,
      S6
    }

    public record Node(Pose3d pose, Side side, Level level, ReefFace reefFace) {}

    public static final List<Node> NODES = buildNodes();

    public static final List<Integer> REEF_TAGS = buildReefTags();

    public static Node getNode(Level level, ReefFace reefFace, Side side) {
      return NODES.stream()
          .filter(node -> node.level == level && node.reefFace == reefFace && node.side == side)
          .findFirst()
          .orElseThrow(() -> new IllegalArgumentException("Unknown reef node"));
    }

    private static List<Integer> buildReefTags() {
      List<Integer> tags = new ArrayList<>();
      for (int i = 17; i <= 22; i++) {
        tags.add(i);
      }
      for (int i = 6; i <= 11; i++) {
        tags.add(i);
      }
      return List.copyOf(tags);
    }

    private static List<Node> buildNodes() {
      List<Node> nodes = new ArrayList<>();
      for (Level level : Level.values()) {
        for (ReefFace face : ReefFace.values()) {
          for (Side side : Side.values()) {
            nodes.add(new Node(calcNodePose(level, face, side), side, level, face));
          }
        }
      }
      return List.copyOf(nodes);
    }

    private static Pose3d calcNodePose(Level level, ReefFace face, Side side) {
      double x = REEF_CENTER_BLUE.getX() - level.getRadiusMeters();
      double y = REEF_CENTER_BLUE.getY();

      if (side == Side.LEFT) {
        y -= SIDE_OFFSET_METERS / 2.0;
      } else if (side == Side.RIGHT) {
        y += SIDE_OFFSET_METERS / 2.0;
      }

      Translation2d translation = new Translation2d(x, y);

      Rotation2d sideDelta = Rotation2d.fromDegrees(60.0 * face.ordinal());

      double dx = translation.getX() - REEF_CENTER_BLUE.getX();
      double dy = translation.getY() - REEF_CENTER_BLUE.getY();
      double rotatedX = (sideDelta.getCos() * dx) - (sideDelta.getSin() * dy) + REEF_CENTER_BLUE.getX();
      double rotatedY = (sideDelta.getSin() * dx) + (sideDelta.getCos() * dy) + REEF_CENTER_BLUE.getY();
      Translation2d rotatedTranslation = new Translation2d(rotatedX, rotatedY);

      return new Pose3d(
          new Translation3d(
              rotatedTranslation.getX(),
              rotatedTranslation.getY(),
              level.getHeightMeters()),
          new Rotation3d(0.0, Units.degreesToRadians(level.getAngleDegrees()), sideDelta.getRadians()));
    }

    private static List<Pose2d> createFloorAlignmentPoses() {
      List<Pose2d> poses = List.of(
          new Pose2d(new Translation2d(3.12, 4.03), Rotation2d.fromDegrees(0.0)),
          new Pose2d(new Translation2d(3.80, 2.84), Rotation2d.fromDegrees(60.0)),
          new Pose2d(new Translation2d(5.17, 2.84), Rotation2d.fromDegrees(120.0)),
          new Pose2d(new Translation2d(5.86, 4.03), Rotation2d.fromDegrees(180.0)),
          new Pose2d(new Translation2d(5.17, 5.21), Rotation2d.fromDegrees(240.0)),
          new Pose2d(new Translation2d(3.80, 5.21), Rotation2d.fromDegrees(300.0)));
      List<Pose2d> nudged = new ArrayList<>(poses.size());
      for (Pose2d pose : poses) {
        nudged.add(GeometryUtil.nudge(pose, Units.inchesToMeters(1.5), 0.0));
      }
      return List.copyOf(nudged);
    }

    private static List<List<Pose2d>> createBetterFloorPoses() {
      List<List<Pose2d>> poses = new ArrayList<>();
      poses.add(List.of(new Pose2d(3.16, 4.05, new Rotation2d()), new Pose2d(3.16, 3.67, new Rotation2d())));
      poses.add(
          List.of(
              new Pose2d(3.78, 2.89, Rotation2d.fromDegrees(60.0)),
              new Pose2d(4.11, 2.71, Rotation2d.fromDegrees(60.0))));
      poses.add(
          List.of(
              new Pose2d(5.14, 2.85, Rotation2d.fromDegrees(120.0)),
              new Pose2d(5.44, 3.04, Rotation2d.fromDegrees(120.0))));
      poses.add(
          List.of(
              new Pose2d(5.82, 4.00, Rotation2d.k180deg),
              new Pose2d(5.81, 4.33, Rotation2d.k180deg)));
      poses.add(
          List.of(
              new Pose2d(5.22, 5.15, Rotation2d.fromDegrees(-120.0)),
              new Pose2d(4.88, 5.32, Rotation2d.fromDegrees(-120.0))));
      return List.copyOf(poses);
    }

    private static void initLogging() {
      Logger.recordOutput("field/reef/reefCenterBlue", Translation2d.struct, REEF_CENTER_BLUE);
      Logger.recordOutput(
          "field/reef/reefZone", GeometryUtil.getCardinals(REEF_ZONE).toArray(new Pose2d[0]));
      Logger.recordOutput(
          "field/reef/nodes",
          NODES.stream().map(Node::pose).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "field/reef/floorPoses", FLOOR_ALIGNMENT_POSES.toArray(new Pose2d[0]));
    }
  }

  public static final class Processor {
    private Processor() {}

    public static final Pose3d PROCESSOR_HOLE_CENTER =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(221.726104 + 14.0),
                0.0,
                Units.inchesToMeters(17.0)),
            new Rotation3d());

    public static final Rectangle2d PROCESSOR_ZONE =
        new Rectangle2d(
            GeometryUtil.nudge(Processor.PROCESSOR_HOLE_CENTER.toPose2d(), Units.inchesToMeters(0.0),
                Units.inchesToMeters(15.0)),
            Units.inchesToMeters((3 * 12) + 10.0),
            Units.inchesToMeters((12 * 12) + 2.5));

    private static void initLogging() {
      Logger.recordOutput(
          "field/processor/processorHoleCenter", Pose3d.struct, PROCESSOR_HOLE_CENTER);
      Logger.recordOutput(
          "field/processor/processorZone",
          GeometryUtil.getCorners(PROCESSOR_ZONE).toArray(new Pose2d[0]));
    }
  }

  public static final class Barge {
    private Barge() {}

    public static final double ALLIANCE_WALL_TO_CAGE_METERS = Units.inchesToMeters(346.891642);

    public static final Pose2d CAGE_1 =
        new Pose2d(
            new Translation2d(ALLIANCE_WALL_TO_CAGE_METERS, Units.inchesToMeters(285.822375)),
            new Rotation2d());
    public static final Pose2d CAGE_2 =
        new Pose2d(
            new Translation2d(ALLIANCE_WALL_TO_CAGE_METERS, Units.inchesToMeters(242.884958)),
            new Rotation2d());
    public static final Pose2d CAGE_3 =
        new Pose2d(
            new Translation2d(ALLIANCE_WALL_TO_CAGE_METERS, Units.inchesToMeters(199.947458)),
            new Rotation2d());

    public static final Rectangle2d CAGE_ZONE =
        new Rectangle2d(
            CAGE_2,
            Units.inchesToMeters((3 * 12) + 10.0),
            Units.inchesToMeters((12 * 12) + 2.5));

    private static void initLogging() {
      Logger.recordOutput("field/barge/cages", CAGE_1, CAGE_2, CAGE_3);
      Logger.recordOutput(
          "field/barge/cageZone",
          GeometryUtil.getCorners(CAGE_ZONE).toArray(new Pose2d[0]));
    }
  }

  public static final class Source {
    private Source() {}

    public static final Pose2d BLUE_TOP_SOURCE_CENTER =
        GeometryUtil.nudge(
            new Pose2d(
                new Translation2d(1.307, 6.986),
                Rotation2d.fromDegrees(-55.0)),
            Units.inchesToMeters(-4.0),
            0.0);

    public static final Pose2d BLUE_BOTTOM_SOURCE_CENTER =
        GeometryUtil.nudge(
            new Pose2d(
                new Translation2d(0.959, 1.304),
                Rotation2d.fromDegrees(55.0)),
            Units.inchesToMeters(-4.0),
            0.0);

    private static void initLogging() {
      Logger.recordOutput(
          "field/source/blueTopSourceCenter", Pose2d.struct, BLUE_TOP_SOURCE_CENTER);
      Logger.recordOutput(
          "field/source/blueBottomSourceCenter", Pose2d.struct, BLUE_BOTTOM_SOURCE_CENTER);
    }
  }
}
