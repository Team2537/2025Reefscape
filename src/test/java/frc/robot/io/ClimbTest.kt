package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.robot.subsystems.climb.ClimbIO
import frc.robot.subsystems.climb.ClimbIOSim
import lib.math.units.into
import lib.math.units.measuredIn
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class ClimbTest {
    var io: ClimbIO? = null
    var inputs: ClimbIO.ClimbArmInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = ClimbIOSim(
            motor = DCMotor.getNEO(1),
            gearing = 75.0,
            moi = KilogramSquareMeters.of(0.004135203),
            armLength = Inches.of(14.0)
        )

        inputs = ClimbIO.ClimbArmInputs()
    }

    @Test
    fun `Arm Hits Min Limit`(){
        io!!.setVoltage(-6.0 measuredIn Volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assertEquals(-45.0, inputs!!.absoluteAngle into Degrees, 1e-3)

        io!!.stop()
    }

    @Test
    fun `Arm Hits Max Limit`(){
        io!!.setVoltage(6.0 measuredIn Volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assertEquals(225.0, inputs!!.absoluteAngle into Degrees, 1e-3)

        io!!.stop()
    }
}