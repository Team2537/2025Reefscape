package frc.robot.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

class ElevatorIOSim(
    private val gearbox: DCMotor = DCMotor.getKrakenX60Foc(2),
    private val gearing: Double,
    private val drumRadius: Distance,
    private val maxHeight: Distance,
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
        maxHeight into Meters, // Maximum height
        true,
        0.0,
    )

    private val controller = PIDController(pidGains.kP, pidGains.kI, pidGains.kD)
    private val ff = ElevatorFeedforward(ffGains.kS, kG, ffGains.kV, ffGains.kA)

    private var isClosedLoop = false

    private val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()

    override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
        elevatorSim.update(0.02)

        inputs.carriageHeight.mut_replace(elevatorSim.positionMeters, Meters)
        inputs.carriageVelocity.mut_replace(elevatorSim.velocityMetersPerSecond, MetersPerSecond)

        inputs.leftMotorVoltage.mut_replace(appliedVoltage)
        inputs.rightMotorVoltage.mut_replace(appliedVoltage)

        inputs.leftStatorCurrent.mut_replace(elevatorSim.currentDrawAmps, Amps)
        inputs.rightStatorCurrent.mut_replace(elevatorSim.currentDrawAmps, Amps)

        if(isClosedLoop) {
            val voltage = controller.calculate(inputs.carriageHeight into Meters)
            elevatorSim.setInputVoltage(voltage + kG)
            appliedVoltage.mut_replace(voltage + kG, Volts)
        }
    }

    override fun setElevatorVoltage(volts: Voltage) {
        elevatorSim.setInputVoltage(volts into Volts)
        appliedVoltage.mut_replace(volts)
        isClosedLoop = false
    }

    override fun setElevatorHeightTarget(height: Distance) {
        isClosedLoop = true
        controller.setpoint = height into Meters
    }

    override fun stop() {
        elevatorSim.setInputVoltage(0.0)
    }
}