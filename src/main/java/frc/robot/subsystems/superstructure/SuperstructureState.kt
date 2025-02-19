package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.swerve.DriveLimits
import lib.math.units.into
import java.nio.ByteBuffer
import java.util.*

data class SuperstructureState(
    val armAngle: Angle,
    val elevatorHeight: Distance,
    val driveLimits: DriveLimits,
    val nextState: Optional<SuperstructureState> = Optional.empty()
) : StructSerializable {
    companion object {
        class SuperstructureStateStruct : Struct<SuperstructureState> {
            override fun getTypeClass(): Class<SuperstructureState> {
                return SuperstructureState::class.java
            }

            override fun getTypeName(): String {
                return "SuperstructureState"
            }

            override fun getSize(): Int {
                return Double.SIZE_BYTES * 2 + DriveLimits.struct.size
            }

            override fun getSchema(): String {
                return "double armAngle;double elevatorHeight;DriveLimits driveLimits"
            }

            override fun pack(bb: ByteBuffer?, value: SuperstructureState?) {
                bb!!.putDouble(value!!.armAngle into Radians)
                bb.putDouble(value.elevatorHeight into Meters)
                DriveLimits.struct.pack(bb, value.driveLimits)
            }

            override fun unpack(bb: ByteBuffer?): SuperstructureState {
                return SuperstructureState(
                    Radians.of(bb!!.getDouble()),
                    Meters.of(bb.getDouble()),
                    DriveLimits.struct.unpack(bb)
                )
            }
        }

        val struct = SuperstructureStateStruct()
    }
}