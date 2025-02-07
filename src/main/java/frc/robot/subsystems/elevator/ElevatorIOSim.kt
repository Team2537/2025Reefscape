package frc.robot.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

class ElevatorIOSim(
    private val gearbox: DCMotor = DCMotor.getKrakenX60Foc(2),
    private val gearing: Double,
    private val drumRadius: Distance,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains,
    private val kG: Double,
    private val mass: Mass
) : ElevatorIO {

    private val elevatorSim = ElevatorSim(
        gearbox,
        gearing,
        mass into Kilograms,
        drumRadius into Meters,
        0.0, // Minimum Height
        Units.inchesToMeters(72.0), // Maximum height
        true,
        0.0,
    )

    private val controller = PIDController(pidGains.kP, pidGains.kI, pidGains.kD)
    private val ff = ElevatorFeedforward(ffGains.kS, kG, ffGains.kV, ffGains.kA)

    override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {

    }

    override fun setElevatorVoltage(volts: Voltage) {
        super.setElevatorVoltage(volts)
    }

    override fun setElevatorHeightTarget(height: Distance) {
        super.setElevatorHeightTarget(height)
    }

    override fun stop() {
        super.stop()
    }
}