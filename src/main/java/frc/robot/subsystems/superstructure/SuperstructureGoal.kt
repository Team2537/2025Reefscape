package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Radian
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import java.nio.ByteBuffer
import java.util.Optional

object SuperstructureGoal {
    data class SuperstructureState(
        val armAngle: Angle,
        val elevatorHeight: Distance,
        val driveLimits: DriveLimits,
        val nextState: Optional<SuperstructureState> = Optional.empty()
    ) : StructSerializable {
        companion object {
            val struct = SuperstructureStateStruct()
        }
    }
    
    data class DriveLimits(
        val maxLinVel: LinearVelocity,
        val maxAngVel: AngularVelocity,
        val maxAccel: LinearAcceleration
    ) : StructSerializable {
        companion object {
            val struct = DriveLimitStruct()
        }
    }
    
    val STOW = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Inches.of(2.75),
        driveLimits = Drivebase.defaultLimits
    )
    
    val L1 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.028),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(STOW)
    )
    
    val L2 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.238),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(STOW)
    )
    
    val L3 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(0.642),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(STOW)
    )
    
    val L4 = SuperstructureState(
        armAngle = Radians.of(0.862),
        elevatorHeight = Meters.of(1.289),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(STOW)
    )
    
    val L1_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.038),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L1)
    )
    
    val L2_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.248),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L2)
    )
    
    val L3_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(0.652),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L3)
    )
    
    val L4_PREP = SuperstructureState(
        armAngle = Degrees.of(90.0),
        elevatorHeight = Meters.of(1.299),
        driveLimits = Drivebase.defaultLimits,
        nextState = Optional.of(L4)
    )
    
    class DriveLimitStruct : Struct<DriveLimits> {
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
}