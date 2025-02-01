package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import lib.math.units.degrees
import lib.math.units.into
import lib.math.units.meters
import lib.math.units.radians

/**
 * A contract for flipping 2-dimensional geometrical objects.
 *
 * There are existing contracts for (no flipping)[None],
 * (reflection across the x-axis)[ReflectX], and (reflection across the y-axis)[ReflectY].
 *
 * @see None
 * @see ReflectX
 * @see ReflectY
 * @see ReflectXY
 * @see LambdaFlipper2d
 */
interface Flipper2d {
    /**
     * Gets the flipped value of an x-coordinate. Coordinate space is
     * specified by WPI-Lib 2-Dimensional Geometry.
     *
     * @param x The x coordinate.
     *
     * @return The flipped x coordinate.
     */
    fun flipX(x: Distance): Distance

    /**
     * Gets the flipped value of a y-coordinate. Coordinate space is
     * specified by WPI-Lib 2-Dimensional Geometry.
     *
     * @param y The y coordinate.
     *
     * @return The flipped y coordinate.
     */
    fun flipY(y: Distance): Distance

    /**
     * Gets the flipped value of a rotation. Coordinate space is
     * specified by WPI-Lib 2-Dimensional Geometry.
     *
     * @param r The rotational distance.
     *
     * @return The flipped rotation.
     */
    fun flipR(r: Angle): Angle

    companion object {
        /** Singleton that just gives any input back.  */
        private val _it: (Any) -> Any = { it }
        /** Casts [_it] to a typed unary function. */
        private fun <T> it(): (T) -> T {
            @Suppress("UNCHECKED_CAST")
            return _it as ((T) -> T)
        }

        /**
         * Creates a [Flipper2d] that just applies a given x operation.
         *
         * @param flipX The operation to apply to x coordinates
         *
         * @return A new `Flipper2d` that applies the given x operation
         * and has no effect on y coordinates or rotations.
         */
        fun justFlipX(flipX: (Distance) -> Distance): Flipper2d {
            return LambdaFlipper2d(xFun = flipX)
        }

        /**
         * Creates a [Flipper2d] that just applies a given y operation.
         *
         * @param flipY The operation to apply to y coordinates
         *
         * @return A new `Flipper2d` that applies the given y operation
         * and has no effect on x coordinates or rotations.
         */
        fun justFlipY(flipY: (Distance) -> Distance): Flipper2d {
            return LambdaFlipper2d(yFun = flipY)
        }

        /**
         * Creates a [Flipper2d] that just applies a given rotation operation.
         *
         * @param flipR The operation to apply to rotations
         *
         * @return A new `Flipper2d` that applies the given rotation operation
         * and has no effect on x or y coordinates.
         */
        fun justFlipRotation(flipR: (Angle) -> Angle): Flipper2d {
            return LambdaFlipper2d(rFun = flipR)
        }

        /**
         * Calculates `a - x` while retaining the unit of `x` and without
         * allocating any "garbage" measures (i.e. measures that are only
         * used in intermediary mathematical steps).
         *
         * @param x The value to flip
         * @param a The constant value, or twice the radius of flipping
         */
        @Suppress("UNCHECKED_CAST")
        private fun <U : Unit, M: Measure<U>> noGarbageFlip(x: M, a: M): M {
            val unit = x.unit()
            return unit.of((a into unit) - x.magnitude()) as M
        }

        private val HALF_CIRCLE = Math.PI.radians
    }

    operator fun invoke(a: Translation2d): Translation2d {
        return Translation2d(flipX(a.x.meters), flipY(a.y.meters))
    }

    operator fun invoke(a: Rotation2d): Rotation2d {
        return Rotation2d(flipR(a.radians.radians))
    }

    operator fun invoke(a: Pose2d): Pose2d {
        return Pose2d(this(a.translation), this(a.rotation))
    }

    operator fun invoke(a: Transform2d): Transform2d {
        return Transform2d(this(a.translation), this(a.rotation))
    }

    operator fun invoke(a: Twist2d): Twist2d {
        return Twist2d(
            flipX(a.dx.meters) into Meters,
            flipY(a.dy.meters) into Meters,
            flipR(a.dtheta.radians) into Radians
        )
    }

    /**
     * A `Flipper2d` that applies no flipping whatsoever. This flipper will
     * **never** allocate memory.
     */
    object None : Flipper2d {
        override fun flipX(x: Distance): Distance = x

        override fun flipY(y: Distance): Distance = y

        override fun flipR(r: Angle): Angle = r

        override fun invoke(a: Translation2d): Translation2d = a
        override fun invoke(a: Rotation2d): Rotation2d = a
        override fun invoke(a: Transform2d): Transform2d = a
        override fun invoke(a: Twist2d): Twist2d = a
        override fun invoke(a: Pose2d): Pose2d = a
    }

    /**
     * A `Flipper2d` that reflects across the x-axis.
     */
    object ReflectX : Flipper2d {
        override fun flipX(x: Distance): Distance = noGarbageFlip(x, FieldConstants.length)

        override fun flipY(y: Distance): Distance = y

        override fun flipR(r: Angle): Angle = -r // rotations are centered on 0, so simple
                                                 // negation does the trick
    }

    /**
     * A `Flipper2d` that reflects across the y-axis.
     */
    object ReflectY : Flipper2d {
        override fun flipX(x: Distance): Distance = x

        override fun flipY(y: Distance): Distance = noGarbageFlip(y, FieldConstants.width)

        override fun flipR(r: Angle): Angle = noGarbageFlip(r, HALF_CIRCLE)
    }

    /**
     * A `Flipper2d` that reflects across the line *`y = x`*.
     */
    object ReflectXY : Flipper2d {
        override fun flipX(x: Distance): Distance = noGarbageFlip(x, FieldConstants.length)

        override fun flipY(y: Distance): Distance = noGarbageFlip(y, FieldConstants.width)

        // a - -r == r + a
        override fun flipR(r: Angle): Angle = r + HALF_CIRCLE
    }

    /**
     * An implementation of a `Flipper2d` that uses given functions to
     * apply flipping.
     *
     * By default, each function returns the same angle,
     * creating a similar effect to [None]. However, this class will
     * **always** create a copy of geometrical objects, whereas [None]
     * **never** does.
     */
    class LambdaFlipper2d(
        private val xFun: (Distance) -> Distance = it(),
        private val yFun: (Distance) -> Distance = it(),
        private val rFun: (Angle) -> Angle = it(),
    ) : Flipper2d {
        override fun flipX(x: Distance): Distance {
            return xFun(x)
        }

        override fun flipY(y: Distance): Distance {
            return yFun(y)
        }

        override fun flipR(r: Angle): Angle {
            return rFun(r)
        }
    }
}