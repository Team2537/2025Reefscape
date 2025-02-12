package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.inches
import lib.math.units.into
import lib.math.units.measuredIn
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class ElevatorTest {
    var io: ElevatorIO? = null
    var inputs: ElevatorIO.ElevatorInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = ElevatorIOSim(
            gearing = 4.0,
            drumRadius = Inches.of(0.88),
            pidGains = PIDGains(),
            ffGains = FeedforwardGains(kV = 1.77),
            kG = 0.19,
            mass = 15.0 measuredIn Pounds,
            maxHeight = 51.0 measuredIn Inches
        )

        inputs = ElevatorIO.ElevatorInputs()
    }

    @Test
    fun `Elevator moves up`(){
        io!!.setElevatorVoltage(6.0 measuredIn Volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }

        assert(inputs!!.carriageHeight > 0.0.inches)
    }
}