package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import frc.robot.subsystems.superstructure.gripper.GripperIO
import frc.robot.subsystems.superstructure.gripper.GripperIOSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into
import lib.math.units.measuredIn
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class GripperTest {
    var io: GripperIO? = null
    var inputs: GripperIO.GripperInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = GripperIOSim(
            DCMotor.getNEO(1),
            1.0,
            PIDGains(),
            FeedforwardGains(kV = 12/5800.0, kA = 0.1)
        )

        inputs = GripperIO.GripperInputs()
    }

    @Test
    fun `Gripper Spins`(){
        io!!.setVoltage(6.0 measuredIn Volts)
        for(i in 0..2000) {
            io!!.updateInputs(inputs!!)
        }

        println(inputs)

        assertEquals(inputs!!.gripperAppliedVoltage into Volts, 6.0, 1e-5)
        assert((inputs!!.gripperVelocity into RotationsPerSecond) > 0.0)
    }
}