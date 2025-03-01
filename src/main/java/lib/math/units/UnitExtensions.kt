@file:Suppress("UNUSED")

package lib.math.units

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.*
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import kotlin.math.absoluteValue

inline val Double.percent: Dimensionless get() = this measuredIn Percent
inline val Double.raw: Dimensionless get() = this measuredIn Value

inline val Double.meters: Distance get() = this measuredIn Meters
inline val Double.inches: Distance get() = this measuredIn Inches
inline val Double.feet: Distance get() = this measuredIn Feet

inline val Double.seconds: Time get() = this measuredIn Seconds
inline val Double.minutes: Time get() = this measuredIn Minutes

inline val Double.radians: Angle get() = this measuredIn Radians
inline val Double.revolutions: Angle get() = this measuredIn Revolutions
inline val Double.rotations: Angle get() = this measuredIn Rotations
inline val Double.degrees: Angle get() = this measuredIn Degrees

inline val Double.metersPerSecond: LinearVelocity get() = this measuredIn MetersPerSecond
inline val Double.feetPerSecond: LinearVelocity get() = this measuredIn FeetPerSecond
inline val Double.inchesPerSecond: LinearVelocity get() = this measuredIn InchesPerSecond

inline val Double.rotationsPerSecond: AngularVelocity get() = this measuredIn RotationsPerSecond
inline val Double.revolutionsPerSecond: AngularVelocity get() = this measuredIn RevolutionsPerSecond
inline val Double.rpm: AngularVelocity get() = this measuredIn RPM
inline val Double.radiansPerSecond: AngularVelocity get() = this measuredIn RadiansPerSecond
inline val Double.degreesPerSecond: AngularVelocity get() = this measuredIn DegreesPerSecond

inline val Double.hertz: Frequency get() = this measuredIn Hertz

inline val Double.metersPerSecondSquared: LinearAcceleration get() = this measuredIn MetersPerSecondPerSecond
inline val Double.metersPerSecondPerSecond: LinearAcceleration get() = this measuredIn MetersPerSecondPerSecond
inline val Double.feetPerSecondSquared: LinearAcceleration get() = this measuredIn FeetPerSecondPerSecond
inline val Double.feetPerSecondPerSecond: LinearAcceleration get() = this measuredIn FeetPerSecondPerSecond
inline val Double.Gs: LinearAcceleration get() = this measuredIn Units.Gs

inline val Double.rotationsPerSecondSquared: AngularAcceleration get() = this measuredIn RotationsPerSecondPerSecond
inline val Double.rotationsPerSecondPerSecond: AngularAcceleration get() = this measuredIn RotationsPerSecondPerSecond
inline val Double.radiansPerSecondSquared: AngularAcceleration get() = this measuredIn RadiansPerSecondPerSecond
inline val Double.radiansPerSecondPerSecond: AngularAcceleration get() = this measuredIn RadiansPerSecondPerSecond
inline val Double.degreesPerSecondSquared: AngularAcceleration get() = this measuredIn DegreesPerSecondPerSecond
inline val Double.degreesPerSecondPerSecond: AngularAcceleration get() = this measuredIn DegreesPerSecondPerSecond

// 100 grams or fewer makes more sense to measure in grams, but otherwise kilograms makes more sense
inline val Double.grams: Mass get() = if (this.absoluteValue < 100.0) this measuredIn Grams else this measuredIn Kilograms
inline val Double.kg: Mass get() = this measuredIn Kilograms
inline val Double.pounds: Mass get() = this measuredIn Pounds
inline val Double.lbs: Mass get() = this measuredIn Pounds
inline val Double.ounces: Mass get() = this measuredIn Ounces
inline val Double.oz: Mass get() = this measuredIn Ounces

inline val Double.newtons: Force get() = this measuredIn Newtons
inline val Double.poundsOfForce: Force get() = this measuredIn PoundsForce
inline val Double.lbsOfForce: Force get() = this measuredIn PoundsForce
inline val Double.ouncesOfForce: Force get() = this measuredIn OuncesForce
inline val Double.ozOfForce: Force get() = this measuredIn OuncesForce

inline val Double.newtonMeters: Torque get() = this measuredIn NewtonMeters
inline val Double.poundFeet: Torque get() = this measuredIn PoundFeet
inline val Double.poundInches: Torque get() = this measuredIn PoundInches
inline val Double.ounceInches: Torque get() = this measuredIn OunceInches

inline val Double.gramMetersPerSecond: LinearMomentum get() = (this / lib.math.units.kilo) measuredIn KilogramMetersPerSecond
inline val Double.kilogramMetersPerSecond: LinearMomentum get() = this measuredIn KilogramMetersPerSecond

inline val Double.gramMetersSquaredPerSecond: AngularMomentum get() = (this / lib.math.units.kilo) measuredIn KilogramMetersSquaredPerSecond
inline val Double.kilogramMetersSquaredPerSecond: AngularMomentum get() = this measuredIn KilogramMetersSquaredPerSecond

inline val Double.gramMetersSquared: MomentOfInertia get() = (this / lib.math.units.kilo) measuredIn KilogramSquareMeters
inline val Double.kilogramMetersSquared: MomentOfInertia get() = this measuredIn KilogramSquareMeters
inline val Double.KgM2: MomentOfInertia get() = this measuredIn KilogramSquareMeters

inline val Double.volts: Voltage get() = this measuredIn Volts

inline val Double.amps: Current get() = this measuredIn Amps

inline val Double.ohms: Resistance get() = this measuredIn Ohms

inline val Double.joules: Energy get() = this measuredIn Joules

inline val Double.watts: Power get() = this measuredIn Watts
inline val Double.horsepower: Power get() = this measuredIn Horsepower
inline val Double.horses: Power get() = this measuredIn Horsepower

inline val Double.kelvin: Temperature get() = this measuredIn Kelvin
inline val Double.celsius: Temperature get() = this measuredIn Celsius
inline val Double.fahrenheit: Temperature get() = this measuredIn Fahrenheit


inline val Int.percent: Dimensionless get() = this.toDouble() measuredIn Percent
inline val Int.raw: Dimensionless get() = this.toDouble() measuredIn Value

inline val Int.meters: Distance get() = this.toDouble() measuredIn Meters
inline val Int.inches: Distance get() = this.toDouble() measuredIn Inches
inline val Int.feet: Distance get() = this.toDouble() measuredIn Feet

inline val Int.seconds: Time get() = this.toDouble() measuredIn Seconds
inline val Int.minutes: Time get() = this.toDouble() measuredIn Minutes

inline val Int.radians: Angle get() = this.toDouble() measuredIn Radians
inline val Int.revolutions: Angle get() = this.toDouble() measuredIn Revolutions
inline val Int.rotations: Angle get() = this.toDouble() measuredIn Rotations
inline val Int.degrees: Angle get() = this.toDouble() measuredIn Degrees

inline val Int.metersPerSecond: LinearVelocity get() = this.toDouble() measuredIn MetersPerSecond
inline val Int.feetPerSecond: LinearVelocity get() = this.toDouble() measuredIn FeetPerSecond
inline val Int.inchesPerSecond: LinearVelocity get() = this.toDouble() measuredIn InchesPerSecond

inline val Int.rotationsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RotationsPerSecond
inline val Int.revolutionsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RevolutionsPerSecond
inline val Int.rpm: AngularVelocity get() = this.toDouble() measuredIn RPM
inline val Int.radiansPerSecond: AngularVelocity get() = this.toDouble() measuredIn RadiansPerSecond
inline val Int.degreesPerSecond: AngularVelocity get() = this.toDouble() measuredIn DegreesPerSecond

inline val Int.hertz: Frequency get() = this.toDouble() measuredIn Hertz

inline val Int.metersPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Int.metersPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Int.feetPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Int.feetPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Int.Gs: LinearAcceleration get() = this.toDouble() measuredIn Units.Gs

inline val Int.rotationsPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Int.rotationsPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Int.radiansPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Int.radiansPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Int.degreesPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond
inline val Int.degreesPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond

// 100 grams or fewer makes more sense to measure in grams, but otherwise kilograms makes more sense
inline val Int.grams: Mass get() = if (this.toDouble().absoluteValue < 100.0) this.toDouble() measuredIn Grams else this.toDouble() measuredIn Kilograms
inline val Int.kg: Mass get() = this.toDouble() measuredIn Kilograms
inline val Int.pounds: Mass get() = this.toDouble() measuredIn Pounds
inline val Int.lbs: Mass get() = this.toDouble() measuredIn Pounds
inline val Int.ounces: Mass get() = this.toDouble() measuredIn Ounces
inline val Int.oz: Mass get() = this.toDouble() measuredIn Ounces

inline val Int.newtons: Force get() = this.toDouble() measuredIn Newtons
inline val Int.poundsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Int.lbsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Int.ouncesOfForce: Force get() = this.toDouble() measuredIn OuncesForce
inline val Int.ozOfForce: Force get() = this.toDouble() measuredIn OuncesForce

inline val Int.newtonMeters: Torque get() = this.toDouble() measuredIn NewtonMeters
inline val Int.poundFeet: Torque get() = this.toDouble() measuredIn PoundFeet
inline val Int.poundInches: Torque get() = this.toDouble() measuredIn PoundInches
inline val Int.ounceInches: Torque get() = this.toDouble() measuredIn OunceInches

inline val Int.gramMetersPerSecond: LinearMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersPerSecond
inline val Int.kilogramMetersPerSecond: LinearMomentum get() = this.toDouble() measuredIn KilogramMetersPerSecond

inline val Int.gramMetersSquaredPerSecond: AngularMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersSquaredPerSecond
inline val Int.kilogramMetersSquaredPerSecond: AngularMomentum get() = this.toDouble() measuredIn KilogramMetersSquaredPerSecond

inline val Int.gramMetersSquared: MomentOfInertia get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramSquareMeters
inline val Int.kilogramMetersSquared: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters
inline val Int.KgM2: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters

inline val Int.volts: Voltage get() = this.toDouble() measuredIn Volts

inline val Int.amps: Current get() = this.toDouble() measuredIn Amps

inline val Int.ohms: Resistance get() = this.toDouble() measuredIn Ohms

inline val Int.joules: Energy get() = this.toDouble() measuredIn Joules

inline val Int.watts: Power get() = this.toDouble() measuredIn Watts
inline val Int.horsepower: Power get() = this.toDouble() measuredIn Horsepower
inline val Int.horses: Power get() = this.toDouble() measuredIn Horsepower

inline val Int.kelvin: Temperature get() = this.toDouble() measuredIn Kelvin
inline val Int.celsius: Temperature get() = this.toDouble() measuredIn Celsius
inline val Int.fahrenheit: Temperature get() = this.toDouble() measuredIn Fahrenheit


inline val Long.percent: Dimensionless get() = this.toDouble() measuredIn Percent
inline val Long.raw: Dimensionless get() = this.toDouble() measuredIn Value

inline val Long.meters: Distance get() = this.toDouble() measuredIn Meters
inline val Long.inches: Distance get() = this.toDouble() measuredIn Inches
inline val Long.feet: Distance get() = this.toDouble() measuredIn Feet

inline val Long.seconds: Time get() = this.toDouble() measuredIn Seconds
inline val Long.minutes: Time get() = this.toDouble() measuredIn Minutes

inline val Long.radians: Angle get() = this.toDouble() measuredIn Radians
inline val Long.revolutions: Angle get() = this.toDouble() measuredIn Revolutions
inline val Long.rotations: Angle get() = this.toDouble() measuredIn Rotations
inline val Long.degrees: Angle get() = this.toDouble() measuredIn Degrees

inline val Long.metersPerSecond: LinearVelocity get() = this.toDouble() measuredIn MetersPerSecond
inline val Long.feetPerSecond: LinearVelocity get() = this.toDouble() measuredIn FeetPerSecond
inline val Long.inchesPerSecond: LinearVelocity get() = this.toDouble() measuredIn InchesPerSecond

inline val Long.rotationsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RotationsPerSecond
inline val Long.revolutionsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RevolutionsPerSecond
inline val Long.rpm: AngularVelocity get() = this.toDouble() measuredIn RPM
inline val Long.radiansPerSecond: AngularVelocity get() = this.toDouble() measuredIn RadiansPerSecond
inline val Long.degreesPerSecond: AngularVelocity get() = this.toDouble() measuredIn DegreesPerSecond

inline val Long.hertz: Frequency get() = this.toDouble() measuredIn Hertz

inline val Long.metersPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Long.metersPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Long.feetPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Long.feetPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Long.Gs: LinearAcceleration get() = this.toDouble() measuredIn Units.Gs

inline val Long.rotationsPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Long.rotationsPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Long.radiansPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Long.radiansPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Long.degreesPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond
inline val Long.degreesPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond

// 100 grams or fewer makes more sense to measure in grams, but otherwise kilograms makes more sense
inline val Long.grams: Mass get() = if (this.absoluteValue < 100L) this.toDouble() measuredIn Grams else this.toDouble() measuredIn Kilograms
inline val Long.kg: Mass get() = this.toDouble() measuredIn Kilograms
inline val Long.pounds: Mass get() = this.toDouble() measuredIn Pounds
inline val Long.lbs: Mass get() = this.toDouble() measuredIn Pounds
inline val Long.ounces: Mass get() = this.toDouble() measuredIn Ounces
inline val Long.oz: Mass get() = this.toDouble() measuredIn Ounces

inline val Long.newtons: Force get() = this.toDouble() measuredIn Newtons
inline val Long.poundsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Long.lbsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Long.ouncesOfForce: Force get() = this.toDouble() measuredIn OuncesForce
inline val Long.ozOfForce: Force get() = this.toDouble() measuredIn OuncesForce

inline val Long.newtonMeters: Torque get() = this.toDouble() measuredIn NewtonMeters
inline val Long.poundFeet: Torque get() = this.toDouble() measuredIn PoundFeet
inline val Long.poundInches: Torque get() = this.toDouble() measuredIn PoundInches
inline val Long.ounceInches: Torque get() = this.toDouble() measuredIn OunceInches

inline val Long.gramMetersPerSecond: LinearMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersPerSecond
inline val Long.kilogramMetersPerSecond: LinearMomentum get() = this.toDouble() measuredIn KilogramMetersPerSecond

inline val Long.gramMetersSquaredPerSecond: AngularMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersSquaredPerSecond
inline val Long.kilogramMetersSquaredPerSecond: AngularMomentum get() = this.toDouble() measuredIn KilogramMetersSquaredPerSecond

inline val Long.gramMetersSquared: MomentOfInertia get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramSquareMeters
inline val Long.kilogramMetersSquared: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters
inline val Long.KgM2: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters

inline val Long.volts: Voltage get() = this.toDouble() measuredIn Volts

inline val Long.amps: Current get() = this.toDouble() measuredIn Amps

inline val Long.ohms: Resistance get() = this.toDouble() measuredIn Ohms

inline val Long.joules: Energy get() = this.toDouble() measuredIn Joules

inline val Long.watts: Power get() = this.toDouble() measuredIn Watts
inline val Long.horsepower: Power get() = this.toDouble() measuredIn Horsepower
inline val Long.horses: Power get() = this.toDouble() measuredIn Horsepower

inline val Long.kelvin: Temperature get() = this.toDouble() measuredIn Kelvin
inline val Long.celsius: Temperature get() = this.toDouble() measuredIn Celsius
inline val Long.fahrenheit: Temperature get() = this.toDouble() measuredIn Fahrenheit


inline val Number.percent: Dimensionless get() = this.toDouble() measuredIn Percent
inline val Number.raw: Dimensionless get() = this.toDouble() measuredIn Value

inline val Number.meters: Distance get() = this.toDouble() measuredIn Meters
inline val Number.inches: Distance get() = this.toDouble() measuredIn Inches
inline val Number.feet: Distance get() = this.toDouble() measuredIn Feet

inline val Number.seconds: Time get() = this.toDouble() measuredIn Seconds
inline val Number.minutes: Time get() = this.toDouble() measuredIn Minutes

inline val Number.radians: Angle get() = this.toDouble() measuredIn Radians
inline val Number.revolutions: Angle get() = this.toDouble() measuredIn Revolutions
inline val Number.rotations: Angle get() = this.toDouble() measuredIn Rotations
inline val Number.degrees: Angle get() = this.toDouble() measuredIn Degrees

inline val Number.metersPerSecond: LinearVelocity get() = this.toDouble() measuredIn MetersPerSecond
inline val Number.feetPerSecond: LinearVelocity get() = this.toDouble() measuredIn FeetPerSecond
inline val Number.inchesPerSecond: LinearVelocity get() = this.toDouble() measuredIn InchesPerSecond

inline val Number.rotationsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RotationsPerSecond
inline val Number.revolutionsPerSecond: AngularVelocity get() = this.toDouble() measuredIn RevolutionsPerSecond
inline val Number.rpm: AngularVelocity get() = this.toDouble() measuredIn RPM
inline val Number.radiansPerSecond: AngularVelocity get() = this.toDouble() measuredIn RadiansPerSecond
inline val Number.degreesPerSecond: AngularVelocity get() = this.toDouble() measuredIn DegreesPerSecond

inline val Number.hertz: Frequency get() = this.toDouble() measuredIn Hertz

inline val Number.metersPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Number.metersPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn MetersPerSecondPerSecond
inline val Number.feetPerSecondSquared: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Number.feetPerSecondPerSecond: LinearAcceleration get() = this.toDouble() measuredIn FeetPerSecondPerSecond
inline val Number.Gs: LinearAcceleration get() = this.toDouble() measuredIn Units.Gs

inline val Number.rotationsPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Number.rotationsPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RotationsPerSecondPerSecond
inline val Number.radiansPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Number.radiansPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn RadiansPerSecondPerSecond
inline val Number.degreesPerSecondSquared: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond
inline val Number.degreesPerSecondPerSecond: AngularAcceleration get() = this.toDouble() measuredIn DegreesPerSecondPerSecond

// 100 grams or fewer makes more sense to measure in grams, but otherwise kilograms makes more sense
inline val Number.grams: Mass get() = if (this.toDouble().absoluteValue < 100.0) this.toDouble() measuredIn Grams else this.toDouble() measuredIn Kilograms
inline val Number.kg: Mass get() = this.toDouble() measuredIn Kilograms
inline val Number.pounds: Mass get() = this.toDouble() measuredIn Pounds
inline val Number.lbs: Mass get() = this.toDouble() measuredIn Pounds
inline val Number.ounces: Mass get() = this.toDouble() measuredIn Ounces
inline val Number.oz: Mass get() = this.toDouble() measuredIn Ounces

inline val Number.newtons: Force get() = this.toDouble() measuredIn Newtons
inline val Number.poundsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Number.lbsOfForce: Force get() = this.toDouble() measuredIn PoundsForce
inline val Number.ouncesOfForce: Force get() = this.toDouble() measuredIn OuncesForce
inline val Number.ozOfForce: Force get() = this.toDouble() measuredIn OuncesForce

inline val Number.newtonMeters: Torque get() = this.toDouble() measuredIn NewtonMeters
inline val Number.poundFeet: Torque get() = this.toDouble() measuredIn PoundFeet
inline val Number.poundInches: Torque get() = this.toDouble() measuredIn PoundInches
inline val Number.ounceInches: Torque get() = this.toDouble() measuredIn OunceInches

inline val Number.gramMetersPerSecond: LinearMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersPerSecond
inline val Number.kilogramMetersPerSecond: LinearMomentum get() = this.toDouble() measuredIn KilogramMetersPerSecond

inline val Number.gramMetersSquaredPerSecond: AngularMomentum get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramMetersSquaredPerSecond
inline val Number.kilogramMetersSquaredPerSecond: AngularMomentum get() = this.toDouble() measuredIn KilogramMetersSquaredPerSecond

inline val Number.gramMetersSquared: MomentOfInertia get() = (this.toDouble() / lib.math.units.kilo) measuredIn KilogramSquareMeters
inline val Number.kilogramMetersSquared: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters
inline val Number.KgM2: MomentOfInertia get() = this.toDouble() measuredIn KilogramSquareMeters

inline val Number.volts: Voltage get() = this.toDouble() measuredIn Volts

inline val Number.amps: Current get() = this.toDouble() measuredIn Amps

inline val Number.ohms: Resistance get() = this.toDouble() measuredIn Ohms

inline val Number.joules: Energy get() = this.toDouble() measuredIn Joules

inline val Number.watts: Power get() = this.toDouble() measuredIn Watts
inline val Number.horsepower: Power get() = this.toDouble() measuredIn Horsepower
inline val Number.horses: Power get() = this.toDouble() measuredIn Horsepower

inline val Number.kelvin: Temperature get() = this.toDouble() measuredIn Kelvin
inline val Number.celsius: Temperature get() = this.toDouble() measuredIn Celsius
inline val Number.fahrenheit: Temperature get() = this.toDouble() measuredIn Fahrenheit

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

// Unfortunately, there is nothing strictly linking the type of
// `Measure<DistanceUnit>` to the type of `Distance`, so specifying
// the specific implementation of `Measure<U>` to return is now
// necessary. Well, not really, but I would rather not return a wildcard
//
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
// As such, there is no longer a generic version of measuredIn. Custom
// units will have to give their own implementation to pair with [Unit.of]


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

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.unaryPlus(): Base {
    return copy()
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.plusAssign(addend: Base) {
    mut_plus(addend)
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.minusAssign(subtrahend: Base) {
    mut_minus(subtrahend)
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.timesAssign(multiplicand: Double) {
    mut_times(multiplicand)
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.timesAssign(multiplicand: Dimensionless) {
    mut_times(multiplicand)
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.divAssign(divisor: Double) {
    mut_divide(divisor)
}

operator fun <U : Unit, Base : Measure<U>, M : MutableMeasure<U, Base, M>> M.divAssign(divisor: Dimensionless) {
    mut_divide(divisor)
}

// Maintain communicative property of multiplication
operator fun <U : Unit, M : Measure<U>> Double.times(m: M): M {
    @Suppress("UNCHECKED_CAST")
    return m.times(this) as M
}

operator fun <U : Unit, M : Measure<U>> Int.times(m: M): M {
    @Suppress("UNCHECKED_CAST")
    return m.times(this.toDouble()) as M
}

operator fun <U : Unit, M : Measure<U>> Long.times(m: M): M {
    @Suppress("UNCHECKED_CAST")
    return m.times(this.toDouble()) as M
}

operator fun <U : Unit, M : Measure<U>> Number.times(m: M): M {
    @Suppress("UNCHECKED_CAST")
    return m.times(this.toDouble()) as M
}

fun <U : Unit, M : Measure<U>> M.epsilonEquals(other: M, epsilon: Double): Boolean {
    return MathUtil.isNear(this.baseUnitMagnitude(), other.baseUnitMagnitude(), epsilon)
}

fun <U : Unit, M : Measure<U>> M.epsilonEquals(other: M, epsilon: M): Boolean {
    return MathUtil.isNear(this.baseUnitMagnitude(), other.baseUnitMagnitude(), epsilon.baseUnitMagnitude())
}

