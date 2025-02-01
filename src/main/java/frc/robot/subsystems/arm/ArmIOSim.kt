package frc.robot.subsystems.arm

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.controllers.gains.ArmFeedforward
import lib.controllers.gains.ControllerGains
import lib.controllers.gains.PIDController
import lib.math.units.into
import lib.math.units.volts

class ArmIOSim(
    motor: DCMotor,
    gearing: Double,
    moi: MomentOfInertia,
    armLength: Distance,
    maxAngle: Angle,
    minAngle: Angle,
    startAngle: Angle,
    gains: ControllerGains,
    gravityGain: Double, // ControllerGains/FeedforwardGains has no kG, so it is tacked on
) : ArmIO {

    private val sim: SingleJointedArmSim = SingleJointedArmSim(
        motor,
        gearing,
        moi into KilogramSquareMeters,
        armLength into Meters,
        minAngle into Radians,
        maxAngle into Radians,
        true,
        startAngle into Radians
    )

    /** PID controller for the pivot arm. */
    private val pid: PIDController = PIDController(gains)
    /** Feedforward controller for the pivot arm. */
    private val feedforward: ArmFeedforward = ArmFeedforward(gains, gravityGain)

    /** Voltage being applied to the motor, stored for logging purposes. */
    private val cachedVoltage: MutVoltage = Volts.zero().mutableCopy()
    /** Whether the voltage is being set by a PID controller. */
    private var isPID = false

    /**
     * Updates the given inputs with data from the arm systems.
     *
     * @param inputs The inputs to update.
     */
    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        inputs.motorRelativePosition.mut_replace(sim.angleRads, Radians)
        inputs.motorAbsolutePosition.mut_replace(sim.angleRads, Radians)
        inputs.motorVelocity.mut_replace(sim.velocityRadPerSec, RadiansPerSecond)
        // FIXME: THIS IS HARDCODED AND SHOULD NOT BE; REPLACE WITH REAL CONSTANT ASAP
        inputs.motorSupplyVoltage.mut_replace(12.volts)
        inputs.motorVoltage.mut_replace(cachedVoltage)
        inputs.motorStatorCurrent.mut_replace(sim.currentDrawAmps, Amps)

        if(isPID){
            setVoltage(Volts.of(pid.calculate(sim.angleRads) + feedforward.calculate(pid.setpoint, 0.0)))
        }

        sim.update(0.02)
    }

    /**
     * Gives the arm a setpoint angle. The given angle is an angle of
     * elevation (relative to the horizontal)
     *
     * @param angle The desired angle of elevation.
     */
    override fun setAngle(angle: Angle) {
        pid.setpoint = angle into Radians
        isPID = true
    }

    /**
     * Sets the motor to run at a specific voltage.
     *
     * @param voltage The desired voltage.
     */
    override fun setVoltage(voltage: Voltage) {
        cachedVoltage.mut_replace(voltage)
        sim.setInputVoltage(voltage into Volts)
    }

    /**
     * Stops the motor.
     */
    override fun stop() {
        isPID = false
        setVoltage(Volts.zero())
    }
}