package frc.robot.subsystems.superstructure.manipulator.wrist

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.math.controllers.gains.PIDGains
import lib.math.units.into
import kotlin.math.PI

class ManipulatorWristSim(
    motor: DCMotor = DCMotor.getKrakenX60Foc(1),
    gearing: Double,
    mass: Mass,
    moi: MomentOfInertia,
    pidGains: PIDGains
): ManipulatorWristIO {
    private val sim: DCMotorSim = DCMotorSim(
        LinearSystemId.createSingleJointedArmSystem(
            motor,
            moi into KilogramSquareMeters,
            gearing
        ),
        motor,
    )

    private val controller = PIDController(pidGains.kP, pidGains.kI, pidGains.kD)
        .apply { enableContinuousInput(0.0, 1.0) }

    private var isClosedLoop = false
    private val targetVoltage: MutVoltage = Volt.zero().mutableCopy()

    override fun updateInputs(inputs: ManipulatorWristIO.ManipulatorWristInputs) {
        if(isClosedLoop){
            val volts = controller.calculate(sim.angularPositionRotations)
            targetVoltage.mut_replace(volts, Volts)
            sim.setInputVoltage(volts)
        }

        sim.update(0.02)

        inputs.angle.mut_replace(sim.angularPosition)
        inputs.velocity.mut_replace(sim.angularVelocity)
        inputs.statorCurrent.mut_replace(sim.currentDrawAmps, Amps)
        inputs.appliedVoltage.mut_replace(targetVoltage)
    }

    override fun setVoltage(voltage: Voltage) {
        sim.setInputVoltage(voltage into Volts)
        targetVoltage.mut_replace(voltage)
    }

    override fun setTargetPosition(targetAngle: Angle) {
        isClosedLoop = true
        controller.setpoint = targetAngle into Rotations
    }
}