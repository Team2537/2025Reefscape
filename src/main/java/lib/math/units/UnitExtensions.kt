package lib.math.units

import edu.wpi.first.units.*
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*

/**
 * Converts this measure to a measure with a different unit of the same type, e.g. minutes to
 * seconds. Converting to the same unit is equivalent to calling [Measure.magnitude].
 *
 * ```kotlin
 *   12.0 measuredIn Meters into Feet // 39.3701
 *   15.0 measuredIn Seconds into Minutes // 0.25
 * ```
 *
 * @param unit The unit to convert this measure to.
 *
 * @return The value of this measure in the given unit.
 *
 * @see [Measure.in]
 */
infix fun <U : Unit> Measure<U>.into(unit: U): Double {
    return this.`in`(unit)
}

/**
 * Measures a double value in a given unit. Will try to give the proper extended
 * type, but may return a generic [Measure] if it is unable to make the determination.
 * See below for some examples of when the compiler is able to make the determination.
 *
 * ```kotlin
 * fun foo(bar: Distance) { // method that has a Distance parameter
 *   println(bar)
 * }
 *
 * val x: Distance = 2.0 measuredIn Meters // x is a Distance
 * foo(x) // all good
 * foo(2.5 measuredIn Feet) // all good
 * val y = 3.4 measuredIn Inches // y is a Measure<DistanceUnit>
 * foo(y) // this will not compile!
 * ```
 */
@Throws(ClassCastException::class)
@Deprecated("Fails to type-check at compile time; deprecated until this is fixed.", replaceWith = ReplaceWith("/* Custom Method Impl */"))
infix fun <U : Unit, R : Measure<U>> Double.measuredIn(unit: U): R {
    // Unfortunately, there is nothing strictly linking the type of
    // `Measure<DistanceUnit>` to the type of `Distance`, so specifying
    // the specific implementation of `Measure<U>` to return is now
    // necessary. Well, not really, but I would rather not return a wildcard

    // FIXME - read this
    // Fortunately, I found an *almost* workaround; with the template arg `R`
    // this method will be able to be inlined as an argument as well as give
    // the proper return when using explicit variable types, e.g.:
    //
    // val x: Distance = 2.0 measuredIn Meters
    // methodThatNeedsDistance(x) // all good
    // methodThatNeedsDistance(2.5 measuredIn Feet) // all good
    // val y = 3.4 measuredIn Inches // y is a Measure<DistanceUnit>
    // methodThatNeedsDistance(y) // this will not compile!
    //
    // There actually is one major problem: this method will not type-check at
    // compile time between different types of `Measure`s, so code such as
    // val z: Distance = 2.0 measuredIn Radians
    // will compile, but will throw a [ClassCastException] the second it
    // gets run.
    @Suppress("UNCHECKED_CAST")
    return unit.of(this) as R
}

// Because `Distance` is a derivation of `Measure<DistanceUnit>`, a type
// of `Measure<DistanceUnit>` can not be given directly to a parameter of
// type `Distance`. The normal solution is that parameters are typed to be
// as high up in the inheritance hierarchy as they can, but here the whole
// point is that you have these types that cause less boilerplate in parameter
// names. Instead, all of that boilerplate is being shunted onto me, here,
// and now.

infix fun Double.measuredIn(unit: DimensionlessUnit): Dimensionless {
    return unit.of(this)
}

/**
 * Measures a distance.
 *
 * @param unit The unit to measure in.
 * @return A measure of the distance.
 */
infix fun Double.measuredIn(unit: DistanceUnit): Distance {
    return unit.of(this)
}

/**
 * Measures a period of time.
 *
 * @param unit The unit to measure in.
 * @return A measure of the time period.
 */
infix fun Double.measuredIn(unit: TimeUnit): Time {
    return unit.of(this)
}

/**
 * Measures an angle.
 *
 * @param unit The unit to measure in.
 * @return A measure of the angle.
 */
infix fun Double.measuredIn(unit: AngleUnit): Angle {
    return unit.of(this)
}

/**
 * Measures a linear velocity.
 *
 * @param unit The unit to measure in.
 * @return A measure of the linear velocity.
 */
infix fun Double.measuredIn(unit: LinearVelocityUnit): LinearVelocity {
    return unit.of(this)
}

/**
 * Measures an angular velocity.
 *
 * @param unit The unit to measure in.
 * @return A measure of the angular velocity.
 */
infix fun Double.measuredIn(unit: AngularVelocityUnit): AngularVelocity {
    return unit.of(this)
}

/**
 * Measures a velocity.
 *
 * @param unit The unit to measure in.
 * @param D The dimension of the velocity (The numerator; Time is the denominator).
 * @return A measure of the velocity.
 */
infix fun <D : Unit> Double.measuredIn(unit: VelocityUnit<D>): Velocity<D> {
    return unit.of(this)
}

/**
 * Measures a frequency.
 *
 * @param unit The unit to measure in.
 * @return A measure of the frequency.
 */
infix fun Double.measuredIn(unit: FrequencyUnit): Frequency {
    return unit.of(this)
}

/**
 * Measures a linear acceleration.
 *
 * @param unit The unit to measure in.
 * @return A measure of the linear acceleration.
 */
infix fun Double.measuredIn(unit: LinearAccelerationUnit): LinearAcceleration {
    return unit.of(this)
}

/**
 * Measures an angular acceleration.
 *
 * @param unit The unit to measure in.
 * @return A measure of the angular acceleration.
 */
infix fun Double.measuredIn(unit: AngularAccelerationUnit): AngularAcceleration {
    return unit.of(this)
}

/**
 * Measures a mass.
 *
 * @param unit The unit to measure in.
 * @return A measure of the mass.
 */
infix fun Double.measuredIn(unit: MassUnit): Mass {
    return unit.of(this)
}

/**
 * Measures a force.
 *
 * @param unit The unit to measure in.
 * @return A measure of the force.
 */
infix fun Double.measuredIn(unit: ForceUnit): Force {
    return unit.of(this)
}

/**
 * Measures a torque.
 *
 * @param unit The unit to measure in.
 * @return A measure of the torque.
 */
infix fun Double.measuredIn(unit: TorqueUnit): Torque {
    return unit.of(this)
}

/**
 * Measures a linear momentum.
 *
 * @param unit The unit to measure in.
 * @return A measure of the linear momentum.
 */
infix fun Double.measuredIn(unit: LinearMomentumUnit): LinearMomentum {
    return unit.of(this)
}

/**
 * Measures an angular momentum.
 *
 * @param unit The unit to measure in.
 * @return A measure of the angular momentum.
 */
infix fun Double.measuredIn(unit: AngularMomentumUnit): AngularMomentum {
    return unit.of(this)
}

/**
 * Measures a moment of inertia.
 *
 * @param unit The unit to measure in.
 * @return A measure of the moment of inertia.
 */
infix fun Double.measuredIn(unit: MomentOfInertiaUnit): MomentOfInertia {
    return unit.of(this)
}

/**
 * Measures a voltage.
 *
 * @param unit The unit to measure in.
 * @return A measure of the voltage.
 */
infix fun Double.measuredIn(unit: VoltageUnit): Voltage {
    return unit.of(this)
}

/**
 * Measures a current.
 *
 * @param unit The unit to measure in.
 * @return A measure of the current.
 */
infix fun Double.measuredIn(unit: CurrentUnit): Current {
    return unit.of(this)
}

/**
 * Measures an amount of resistance.
 *
 * @param unit The unit to measure in.
 * @return A measure of the resistance.
 */
infix fun Double.measuredIn(unit: ResistanceUnit): Resistance {
    return unit.of(this)
}

/**
 * Measures an amount of energy.
 *
 * @param unit The unit to measure in.
 * @return A measure of the energy.
 */
infix fun Double.measuredIn(unit: EnergyUnit): Energy {
    return unit.of(this)
}

/**
 * Measures an amount of power.
 *
 * @param unit The unit to measure in.
 * @return A measure of the power.
 */
infix fun Double.measuredIn(unit: PowerUnit): Power {
    return unit.of(this)
}

/**
 * Measures a temperature.
 *
 * @param unit The unit to measure in.
 * @return A measure of the temperature.
 */
infix fun Double.measuredIn(unit: TemperatureUnit): Temperature {
    return unit.of(this)
}

/**
 * Measures an arbitrary [Per].
 *
 * @param unit The unit to measure in.
 * @param N The numerator.
 * @param D The denominator.
 * @return A measure of the relationship of the two given units.
 */
infix fun <N : Unit, D : Unit> Double.measuredIn(unit: PerUnit<N, D>): Per<N, D> {
    @Suppress("UNCHECKED_CAST")
    return unit.of(this) as Per<N, D>
}

/**
 * Measures an arbitrary [Per].
 *
 * @param unit The unit to measure in.
 * @param A The first unit.
 * @param B The other unit.
 * @return A measure of the relationship of the two given units.
 */
infix fun <A : Unit, B : Unit> Double.measuredIn(unit: MultUnit<A, B>): Mult<A, B> {
    @Suppress("UNCHECKED_CAST")
    return unit.of(this) as Mult<A, B>
}
