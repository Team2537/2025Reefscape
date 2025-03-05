package lib.controllers.pathfollowing.repulsor

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.interpolation.Interpolatable
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutDistance
import lib.math.units.into
import lib.math.units.meters
import kotlin.math.hypot

data class Force(val x: Double, val y: Double) : Interpolatable<Force> {
    
    constructor() : this(0.0, 0.0)
    constructor(x: Distance, y: Distance) : this(x into Meters, y into Meters)
    constructor(vec: Vector<N2>) : this(vec[0], vec[1])
    constructor(magnitude: Double, angle: Rotation2d) : this(magnitude * angle.cos, magnitude * angle.sin)
    constructor(magnitude: Distance, angle: Rotation2d) : this(magnitude into Meters, angle)
    
    val measureX: MutDistance = x.meters.mutableCopy()
        get() = field.mut_replace(x, Meters)
    
    val measureY: MutDistance = y.meters.mutableCopy()
        get() = field.mut_replace(y, Meters)
    
    val norm get() = hypot(x, y)
    val angle get() = Rotation2d(x, y)
    
    fun toVector(): Vector<N2> {
        return VecBuilder.fill(x, y)
    }
    
    fun getDistance(other: Force): Double {
        return hypot(other.x - x, other.y - y)
    }
    
    fun rotateBy(angle: Rotation2d): Force {
        return Force(
            x * angle.cos - y * angle.sin,
            x * angle.sin + y * angle.cos
        )
    }
    
    fun rotateAround(other: Force, angle: Rotation2d): Force {
        return Force(
            other.x + (x - other.x) * angle.cos - (y - other.y) * angle.sin,
            other.y + (x - other.x) * angle.sin + (y - other.y) * angle.cos
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
    
    override fun interpolate(endValue: Force, t: Double): Force {
        return Force(
            MathUtil.interpolate(x, endValue.x, t),
            MathUtil.interpolate(y, endValue.y, t)
        )
    }
    
    companion object {
        val ZERO = Force()
        val IHAT = Force(1.0, 0.0)
        val JHAT = Force(0.0, 1.0)
    }
}