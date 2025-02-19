package frc.robot.subsystems.swerve

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import lib.math.units.into
import java.nio.ByteBuffer

data class DriveLimits(
    val maxLinVel: LinearVelocity,
    val maxAngVel: AngularVelocity,
    val maxAccel: LinearAcceleration
) : StructSerializable {
    companion object {
        class DriveLimitsStruct : Struct<DriveLimits> {
            override fun getTypeName(): String {
                return "DriveLimits"
            }

            override fun getTypeClass(): Class<DriveLimits> {
                return DriveLimits::class.java
            }

            override fun getSize(): Int {
                return Double.SIZE_BYTES * 3
            }

            override fun getSchema(): String {
                return "double maxLinVel;double maxAngVel;double maxAccel"
            }

            override fun unpack(bb: ByteBuffer?): DriveLimits {
                return DriveLimits(
                    MetersPerSecond.of(bb!!.getDouble()),
                    RadiansPerSecond.of(bb.getDouble()),
                    MetersPerSecondPerSecond.of(bb.getDouble())
                )
            }

            override fun pack(bb: ByteBuffer?, value: DriveLimits?) {
                bb!!.putDouble(value!!.maxLinVel into MetersPerSecond)
                bb.putDouble(value.maxAngVel into RadiansPerSecond)
                bb.putDouble(value.maxAccel into MetersPerSecondPerSecond)
            }
        }

        val struct = DriveLimitsStruct()
    }
}