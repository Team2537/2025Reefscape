package frc.robot

import edu.wpi.first.units.Units
import lib.math.units.PoundSquareFeet
import lib.math.units.into
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

internal class MomentOfIntertiaTest {
    @Test
    fun `Pound Squared Feet Test`(){
        val testValue = Units.KilogramSquareMeters.of(12.5)

        assertEquals(296.62950505, testValue into PoundSquareFeet, 1E-6)
    }
}