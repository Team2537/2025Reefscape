package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOSim
import frc.robot.subsystems.intake.roller.IntakeRollerIO
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.measuredIn
import lib.math.units.radiansPerSecond
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

internal class IntakeRollerTest {
    var io: IntakeRollerIO? = null
    var inputs: IntakeRollerIO.IntakeRollerInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = IntakeRollerIOSim(
            DCMotor.getNEO(1),
            gearing = 1.0,
            moi = KilogramSquareMeters.of(0.000033228)
        )

        inputs = IntakeRollerIO.IntakeRollerInputs()
    }

    @Test
    fun `Spin rollers`(){
        io!!.setVoltage(6.0 measuredIn Volts)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }

        assert(inputs!!.velocity > 0.0.radiansPerSecond)
    }
}