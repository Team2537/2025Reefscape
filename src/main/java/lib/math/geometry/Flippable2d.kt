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
 * [ClassCastException] being thrown.
 *
 * Based on team 686 "Bovine Intervention"'s [`FlippedGeometry`](https://github.com/FRC-686-Bovine-Intervention/Robot-2025-Reefscape/blob/gamepiece-vis/src/main/java/frc/util/AllianceFlipUtil.java#L188)
 */
class Flippable2d<T> {
    // TODO: maybe with private constructors for type safety? unsure...
    // I will make the rest of these factory methods later if so
    companion object {
        fun withBlue(blue: Translation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Translation2d> {
            return Flippable2d(blue, flip(blue))
        }

        fun withRed(red: Translation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Translation2d> {
            return Flippable2d(flip(red), red)
        }

        fun withBlue(blue: Rotation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Rotation2d> {
            return Flippable2d(blue, flip(blue))
        }

        fun withRed(red: Rotation2d, flip: Flipper2d = Flipper2d.ReflectX): Flippable2d<Rotation2d> {
            return Flippable2d(flip(red), red)
        }
    }

    constructor(raw: T, flip: Flipper2d = Flipper2d.None) {
        blue = raw
        red = flip(blue)
    }

    private constructor(blue: T, red: T) {
        this.blue = blue
        this.red = red
    }

    val blue: T
    val red: T

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