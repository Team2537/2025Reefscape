package lib.math.units

import edu.wpi.first.units.MomentOfInertiaUnit
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.derive
import edu.wpi.first.units.measure.MomentOfInertia

val PoundSquareFeet: MomentOfInertiaUnit =
    derive(KilogramSquareMeters)
        .aggregate(0.0421401101)
        .symbol("lb*ft^2")
        .named("Pound Square Feet")
        .make()

inline val Double.poundFeetSquared: MomentOfInertia get() = this measuredIn PoundSquareFeet
inline val Double.lbFt2: MomentOfInertia get() = this measuredIn PoundSquareFeet
inline val Int.poundFeetSquared: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet
inline val Int.lbFt2: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet
inline val Long.poundFeetSquared: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet
inline val Long.lbFt2: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet
inline val Number.poundFeetSquared: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet
inline val Number.lbFt2: MomentOfInertia get() = this.toDouble() measuredIn PoundSquareFeet