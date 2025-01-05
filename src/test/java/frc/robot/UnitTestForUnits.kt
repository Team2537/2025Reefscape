package frc.robot

import edu.wpi.first.units.Units.*
import lib.math.units.*
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class UnitTestForUnits {
    @Test
    fun `Randoms Test`() {
        val meters = 4567.meters
        val km = 4.567.kilo.meters

        assert(meters.isEquivalent(km))

        val feet = 9.0.deca.feet
        val inches = 12 * 90.inches

        assertEquals(feet into Feet, inches into Feet, 1e-5)

        val t = 3.seconds
        val x = 9.meters

        val s = x / t
        val s2 = x * (1.0 / 3.0).hertz
        assertEquals(s into MetersPerSecond, 3.0, 1e-5)
        assertEquals(s2 into MetersPerSecond, 3.0, 1e-5)
    }

    @Test
    fun `Unit Conversion Test`() {
        val l2ad = linearToAngularD(2.0.meters, 0.5.meters)
        assertEquals(4.radians, l2ad)

        val a2ld = angularToLinearD(4.radians, 0.5.meters)
        assertEquals(2.meters, a2ld)

        val l2av = linearToAngularV(2.0.metersPerSecond, 0.4.meters)
        assertEquals(5.radiansPerSecond, l2av)

        val a2lv = angularToLinearV(4.radiansPerSecond, 2.5.meters)
        assertEquals(10.metersPerSecond, a2lv)
    }
}