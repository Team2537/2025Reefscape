package frc.robot.io

import edu.wpi.first.hal.HAL
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.robot.subsystems.superstructure.arm.ArmIO
import frc.robot.subsystems.superstructure.arm.ArmIOSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.degrees
import lib.math.units.into
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals


internal class ArmTest {
    var io: ArmIO? = null
    var inputs: ArmIO.ArmInputs? = null

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))

        io = ArmIOSim(
            motor = DCMotor.getKrakenX60Foc(1),
            gearing = 50.0,
            moi = KilogramSquareMeters.of(0.164586918),
            pidGains = PIDGains(kP = 20.0),
            ffGains = FeedforwardGains(kV = 0.99),
            kG = 0.21
        )

        inputs = ArmIO.ArmInputs()
    }

    @Test
    fun `Arm spins when powered`(){
        io!!.setVoltage(Volts.of(6.0))
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assert(inputs!!.motorAbsolutePosition > 0.0.degrees)
    }


    @Test
    fun `Arm tracks setpoint`(){
        // Does the arm get within 5 degrees of the setpoint?
        io!!.setAngle(90.0.degrees)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assertEquals(90.0, inputs!!.motorAbsolutePosition into Degrees, 5.0)
        // Run longer to see if it stays within 5 degrees
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assertEquals(90.0, inputs!!.motorAbsolutePosition into Degrees, 5.0)
        // Run with a different setpoint
        io!!.setAngle(180.0.degrees)
        for(i in 0..2000){
            io!!.updateInputs(inputs!!)
        }
        assertEquals(180.0, inputs!!.motorAbsolutePosition into Degrees, 5.0)
    }
}