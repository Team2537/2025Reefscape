package lib.controllers.pathfollowing.repulsor

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.interpolation.Interpolatable
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.protobuf.ProtobufSerializable
import edu.wpi.first.util.struct.StructSerializable
import lib.math.units.into
import kotlin.math.hypot

/**
 * Represents a 2D force vector.
 * @param x The x component of the force.
 * @param y The y component of the force.
 */
class Force(
    val x: Double,
    val y: Double,
) : Interpolatable<Force>, ProtobufSerializable, StructSerializable {
    /**
     * Constructs a force with both components set to zero.
     */
    constructor() : this(0.0, 0.0)

    /**
     * Constructs a force with the given components.
     * @param x The x component of the force.
     * @param y The y component of the force.
     */
    constructor(x: Distance, y: Distance) : this(x into Meters, y into Meters)

    /**
     * Constructs a force using polar coordinates.
     * @param distance The magnitude of the force.
     * @param angle The angle of the force.
     */
    constructor(distance: Double, angle: Rotation2d) : this(
        distance * angle.cos,
        distance * angle.sin,
    )

    /**
     * Constructs a force using a vector.
     * @param vector The vector to use.
     */
    constructor(vector: Vector<N2>) : this(vector[0], vector[1])

    val norm: Double
        get() = hypot(x, y)

    val angle: Rotation2d
        get() = Rotation2d(x, y)

    /**
     * Gets the distance between this force and another force.
     * @param other The other force.
     */
    fun getDistance(other: Force): Double {
        return hypot(other.x - x, other.y - y)
    }

    /**
     * Applies a rotation to the force.
     */
    fun rotate(angle: Rotation2d): Force {
        return Force(
            angle.cos * x - angle.sin * y,
            angle.sin * x + angle.cos * y,
        )
    }

    fun rotateAround(
        rot: Rotation2d,
        other: Force,
    ): Force {
        return Force(
            (x - other.x) * rot.cos - (y - other.y) * rot.sin + other.x,
            (x - other.x) * rot.sin + (y - other.y) * rot.cos + other.y,
        )
    }

    operator fun plus(other: Force): Force {
        return Force(x + other.x, y + other.y)
    }

    operator fun minus(other: Force): Force {
        return Force(x - other.x, y - other.y)
    }

    operator fun times(scalar: Double): Force {
        return Force(x * scalar, y * scalar)
    }

    operator fun div(scalar: Double): Force {
        return Force(x / scalar, y / scalar)
    }

    operator fun unaryMinus(): Force {
        return Force(-x, -y)
    }

    fun nearest(forces: List<Force>): Force {
        return forces.minByOrNull { getDistance(it) } ?: Force.ZERO
    }

    override fun interpolate(
        endValue: Force,
        t: Double,
    ): Force {
        return Force(
            MathUtil.interpolate(x, endValue.x, t),
            MathUtil.interpolate(y, endValue.y, t),
        )
    }

    companion object {
        val ZERO = Force()
    }
}
