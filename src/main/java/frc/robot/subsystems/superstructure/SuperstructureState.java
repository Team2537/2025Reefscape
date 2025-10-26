package frc.robot.subsystems.superstructure;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.swerve.Drivebase;
import java.nio.ByteBuffer;
import java.util.Optional;

/** Immutable description of a superstructure target state. */
public final class SuperstructureState implements StructSerializable {
  public static final Struct<SuperstructureState> STRUCT = new SuperstructureStateStruct();

  private final Rotation2d armAngle;
  private final PathConstraints driveLimits;
  private final Optional<SuperstructureState> nextState;
  private final String name;

  public SuperstructureState(Rotation2d armAngle) {
    this(armAngle, Drivebase.getDefaultLimits(), Optional.empty(), "");
  }

  public SuperstructureState(
      Rotation2d armAngle,
      PathConstraints driveLimits,
      Optional<SuperstructureState> nextState,
      String name) {
    this.armAngle = armAngle;
    this.driveLimits = driveLimits;
    this.nextState = nextState == null ? Optional.empty() : nextState;
    this.name = name;
  }

  public Rotation2d getArmAngle() {
    return armAngle;
  }

  public PathConstraints getDriveLimits() {
    return driveLimits;
  }

  public Optional<SuperstructureState> getNextState() {
    return nextState;
  }

  public String getName() {
    return name;
  }

  @Override
  public Struct<SuperstructureState> getStruct() {
    return STRUCT;
  }

  private static final class SuperstructureStateStruct implements Struct<SuperstructureState> {
    @Override
    public Class<SuperstructureState> getTypeClass() {
      return SuperstructureState.class;
    }

    @Override
    public String getTypeName() {
      return "SuperstructureState";
    }

    @Override
    public int getSize() {
      return Double.BYTES;
    }

    @Override
    public String getSchema() {
      return "double armAngle;";
    }

    @Override
    public void pack(ByteBuffer bb, SuperstructureState value) {
      bb.putDouble(value.armAngle.getRadians());
    }

    @Override
    public SuperstructureState unpack(ByteBuffer bb) {
      double angleRad = bb.getDouble();
      return new SuperstructureState(Rotation2d.fromRadians(angleRad));
    }
  }
}
