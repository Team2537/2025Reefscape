package lib.math.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import lib.math.geometry.JavaDriverStation.*

class Flippable2d<T>(raw: T, flip: Flipper2d = Flipper2d.None) {
    enum class Side {
        RED,
        BLUE,
    }

    var blue: T private set
    var red: T private set

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

    init {
        // TODO: find a better and more general solution to this
        if(
            raw !is Translation2d && raw !is Rotation2d && raw !is Pose2d &&
            raw !is Transform2d && raw !is Twist2d
        ) {
            throw ClassCastException("Cannot have flippable ${raw!!::class.qualifiedName}")
        }

        blue = raw
        red = flip(blue)
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
    operator fun <T> Flipper2d.invoke(obj: T): T {
        @Suppress("UNNECESSARY_NOT_NULL_ASSERTION")
        return when(obj) {
            null -> null as T
            is Translation2d -> this(obj) as T
            is Rotation2d -> this(obj) as T
            is Pose2d -> this(obj) as T
            is Transform2d -> this(obj) as T
            is Twist2d -> this(obj) as T
            // The non-null assertion is needed for this to compile.
            // The non-null assertion is not needed as we have already checked for null
            // The non-null assertion is here to stay
            else -> throw ClassCastException("Cannot flip type ${obj!!::class.qualifiedName}")
        }
    }

}