package frc.robot.subsystems.superstructure

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import java.nio.ByteBuffer
import java.util.*

data class SuperstructureState(
    val armAngle: Angle,
    val driveLimits: PathConstraints = Drivebase.defaultLimits,
    val nextState: Optional<SuperstructureState> = Optional.empty(),
    val name: String = ""
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
                return Double.SIZE_BYTES
            }

            override fun getSchema(): String {
                return "double armAngle;"
            }

            override fun pack(bb: ByteBuffer?, value: SuperstructureState?) {
                bb!!.putDouble(value!!.armAngle into Radians)
            }

            override fun unpack(bb: ByteBuffer?): SuperstructureState {
                return SuperstructureState(
                    Radians.of(bb!!.getDouble()),
                )
            }
        }

        val struct = SuperstructureStateStruct()
    }
}