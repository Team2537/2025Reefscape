package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.KilogramSquareMeters
import frc.robot.subsystems.intake.pivot.IntakePivotIO
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim
import frc.robot.subsystems.intake.roller.IntakeRollerIO
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into
import lib.math.units.measuredIn
import lib.math.units.volts
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals

internal class IntakePivotTest {
    var io: IntakePivotIO? = null
    var inputs: IntakePivotIO.IntakePivotInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = IntakePivotIOSim(
            motor = DCMotor.getKrakenX60Foc(1),
            gearing = 13.33,
            moi = 1.0 measuredIn KilogramSquareMeters,
            pidGains = PIDGains(),
            ffGains = FeedforwardGains(),
            kG = 0.0
        )

        inputs = IntakePivotIO.IntakePivotInputs()
    }

    @Test
    fun `Intake pivot hits top limit`(){
        io!!.setVoltage(6.0.volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }

        assertEquals(90.0, inputs!!.position into Degrees, 1e-3)
    }

    @Test
    fun `Intake pivot hits bottom limit`() {
        io!!.setVoltage((-6.0).volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }

        assertEquals(45.0, inputs!!.position into Degrees, 1e-3)
    }
}