package frc.robot.subsystems.superstructure;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.subsystems.swerve.Drivebase;
import lib.math.units.into;
import java.nio.ByteBuffer;
import java.util.*;

public class SuperstructureState implements StructSerializable {
    public final Angle armAngle;
    public final PathConstraints driveLimits;
    public final Optional<SuperstructureState> nextState;
    public final String name;

    public SuperstructureState(
        Angle armAngle,
        PathConstraints driveLimits,
        String name
    ) {
        this.armAngle = armAngle;
        this.driveLimits = driveLimits;
        this.nextState = Optional.empty();
        this.name = name;
    }

    public SuperstructureState(
        Angle armAngle,
        PathConstraints driveLimits,
        Optional<SuperstructureState> nextState,
        String name
    ) {
        this.armAngle = armAngle;
        this.driveLimits = driveLimits;
        this.nextState = nextState;
        this.name = name;
    }

    public SuperstructureState(
        Angle armAngle
    ) {
        this.armAngle = armAngle;
        this.driveLimits = Drivebase.defaultLimits;
        this.nextState = Optional.empty();
        this.name = "";
    }

    public static class SuperstructureStateStruct implements Struct<SuperstructureState> {
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
            return Double.SIZE_BYTES;
        }

        @Override
        public String getSchema() {
            return "double armAngle;";
        }

        @Override
        public void pack(ByteBuffer bb, SuperstructureState value) {
            bb.putDouble(value.armAngle.getValue(Radians));
        }

        @Override
        public SuperstructureState unpack(ByteBuffer bb) {
            return new SuperstructureState(
                Radians.of(bb.getDouble())
            );
        }
    }

    public static final SuperstructureStateStruct struct = new SuperstructureStateStruct();
}
