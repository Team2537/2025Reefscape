package frc.robot.subsystems.swerve

import edu.wpi.first.units.Units.*
import edu.wpi.first.util.struct.Struct
import lib.math.units.into
import java.nio.ByteBuffer

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