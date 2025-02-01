package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import lib.math.geometry.JavaDriverStation.*

/**
 * A [Flippable2d] is a type of geometry object that can be flipped across
 * the field in various ways.
 *
 * By default, a [Flippable2d] will [not have any flipping][Flipper2d.None]
 * -- i.e. [blue] and [red] will be the same object (`blue === red`).
 *
 * Flippable geometry can also be accessed with relative respect through
 * the use of [ours] and [theirs]. Note that "our" side will be blue if
 * there is not a valid alliance found.
 *
 * Note that there is no bounding on the type parameter of a [Flippable2d].
 * Attempting to use a type that cannot be flipped will result in a
 * [ClassCastException] being thrown. For convenience, static factory
 * methods exist only for currently supported geometrical types.
 *
 * Based on team 686 "Bovine Intervention"'s [`FlippedGeometry`](https://github.com/FRC-686-Bovine-Intervention/Robot-2025-Reefscape/blob/gamepiece-vis/src/main/java/frc/util/AllianceFlipUtil.java#L188)
 */
class Flippable2d<out T> private constructor(val blue: T, val red: T) {
    companion object {
        /**
         * Creates a `Flippable2d<Translation2d>` from the given blue [Translation2d].
         *
         * By default, red's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param blue The blue [Translation2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Translation2d>` derived from a blue [Translation2d].
         */
        fun withBlue(blue: Translation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Translation2d> {
            return Flippable2d(blue, flip(blue))
        }

        /**
         * Creates a `Flippable2d<Translation2d>` from the given red [Translation2d].
         *
         * By default, blue's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param red The red [Translation2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Translation2d>` derived from a red [Translation2d].
         */
        fun withRed(red: Translation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Translation2d> {
            return Flippable2d(flip(red), red)
        }

        /**
         * Creates a `Flippable2d<Rotation2d>` from the given blue [Rotation2d].
         *
         * By default, red's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param blue The blue [Rotation2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Rotation2d>` derived from a blue [Rotation2d].
         */
        fun withBlue(blue: Rotation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Rotation2d> {
            return Flippable2d(blue, flip(blue))
        }

        /**
         * Creates a `Flippable2d<Rotation2d>` from the given red [Rotation2d].
         *
         * By default, blue's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param red The red [Rotation2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Rotation2d>` derived from a red [Rotation2d].
         */
        fun withRed(red: Rotation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Rotation2d> {
            return Flippable2d(flip(red), red)
        }

        /**
         * Creates a `Flippable2d<Pose2d>` from the given blue [Pose2d].
         *
         * By default, red's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param blue The blue [Pose2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Pose2d>` derived from a blue [Pose2d].
         */
        fun withBlue(blue: Pose2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Pose2d> {
            return Flippable2d(blue, flip(blue))
        }

        /**
         * Creates a `Flippable2d<Pose2d>` from the given red [Pose2d].
         *
         * By default, blue's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param red The red [Pose2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Pose2d>` derived from a red [Pose2d].
         */
        fun withRed(red: Pose2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Pose2d> {
            return Flippable2d(flip(red), red)
        }

        /**
         * Creates a `Flippable2d<Transform2d>` from the given blue [Transform2d].
         *
         * By default, red's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param blue The blue [Transform2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Transform2d>` derived from a blue [Transform2d].
         */
        fun withBlue(blue: Transform2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Transform2d> {
            return Flippable2d(blue, flip(blue))
        }

        /**
         * Creates a `Flippable2d<Transform2d>` from the given red [Transform2d].
         *
         * By default, blue's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param red The red [Transform2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Transform2d>` derived from a red [Transform2d].
         */
        fun withRed(red: Transform2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Transform2d> {
            return Flippable2d(flip(red), red)
        }

        /**
         * Creates a `Flippable2d<Twist2d>` from the given blue [Twist2d].
         *
         * By default, red's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param blue The blue [Twist2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Twist2d>` derived from a blue [Twist2d].
         */
        fun withBlue(blue: Twist2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Twist2d> {
            return Flippable2d(blue, flip(blue))
        }

        /**
         * Creates a `Flippable2d<Twist2d>` from the given red [Twist2d].
         *
         * By default, blue's version is [reflected across the x-axis][Flipper2d.ReflectX].
         *
         * @param red The red [Twist2d]
         * @param flip The [flipping contract][Flipper2d]
         *
         * @return A `Flippable2d<Twist2d>` derived from a red [Twist2d].
         */
        fun withRed(red: Twist2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Twist2d> {
            return Flippable2d(flip(red), red)
        }
    }

    val ours: T get() {
        return if(DriverStation.alliance == Red)
            red
        else
            blue
    }

    val theirs: T get() {
        return if(DriverStation.alliance == Red)
            blue
        else
            red
    }

    // extension so that it may not be overridden
    /**
     * Attempts to invoke a [Flipper2d] on an arbitrary object.
     * Will throw if it cannot.
     *
     * @param obj The arbitrary object.
     *
     * @return The flipped object if flipping was successful
     *
     * @throws ClassCastException If the method is unable to flip the given object.
     */
    @Throws(ClassCastException::class)
    @Suppress("UNCHECKED_CAST")
    operator fun <U> Flipper2d.invoke(obj: U): U {
        // NOTE: This switch will need to be updated if ever there are other
        // geometry classes we want to flip.
        @Suppress("UNNECESSARY_NOT_NULL_ASSERTION")
        return when(obj) {
            null -> null as U
            is Translation2d -> this(obj) as U
            is Rotation2d -> this(obj) as U
            is Pose2d -> this(obj) as U
            is Transform2d -> this(obj) as U
            is Twist2d -> this(obj) as U
            // The non-null assertion is needed for this to compile.
            // The non-null assertion is not needed as we have already checked for null
            // The non-null assertion is here to stay
            else -> throw ClassCastException("Cannot flip type ${obj!!::class.qualifiedName}")
        }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Flippable2d<*>

        if (blue != other.blue) return false
        if (red != other.red) return false

        return true
    }

    override fun hashCode(): Int {
        var result = blue?.hashCode() ?: 0
        result = 31 * result + (red?.hashCode() ?: 0)
        return result
    }

    override fun toString(): String {
        return "Flippable2d(blue=$blue, red=$red)"
    }
}