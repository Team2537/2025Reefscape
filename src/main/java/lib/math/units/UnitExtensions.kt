package lib.math.units

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

infix fun <U : Unit> Measure<U>.into(unit: U): Double {
    return this.`in`(unit)
}

infix fun <U : Unit> Double.measuredIn(unit: U): Measure<*>? {
    return unit.of(this)
}
