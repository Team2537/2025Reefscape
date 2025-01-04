package lib.math.units

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity

fun linearToAngularV(
    velocity: LinearVelocity,
    radius: Distance,
): AngularVelocity {
    return ((velocity into MetersPerSecond) / (radius into Meters) measuredIn RadiansPerSecond) as AngularVelocity
}

fun linearToAngularV(
    velocityMetersPerSecond: Double,
    radius: Distance,
): AngularVelocity {
    return (velocityMetersPerSecond / (radius into Meters) measuredIn RadiansPerSecond) as AngularVelocity
}

fun angularToLinearV(
    velocity: AngularVelocity,
    radius: Distance,
): LinearVelocity {
    return ((velocity into RadiansPerSecond) * (radius into Meters) measuredIn MetersPerSecond) as LinearVelocity
}

fun linearToAngularD(
    distance: Distance,
    radius: Distance,
): Angle {
    return ((distance into Meters) / (radius into Meters) measuredIn Radians) as Angle
}

fun angularToLinearD(
    angle: Angle,
    radius: Distance,
): Distance {
    return ((angle into Radians) * (radius into Meters) measuredIn Meters) as Distance
}

fun linearToAngularD(
    distanceMeters: Double,
    radius: Distance,
): Distance {
    return (distanceMeters / (radius into Meters) measuredIn Meters) as Distance
}

fun angularToLinearD(
    angleRadians: Double,
    radius: Distance,
): Angle {
    return ((angleRadians) * (radius into Meters) measuredIn Radians) as Angle
}
