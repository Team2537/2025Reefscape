package lib.math.units

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.derive

val PoundSquareFeet =
    derive(KilogramSquareMeters)
        .aggregate(0.0421401101)
        .symbol("lb*ft^2")
        .named("Pound Square Feet")
        .make()