package frc.robot.subsystems.climb

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.math.units.into
import kotlin.math.PI

class ClimbIOSim(
    motor: DCMotor,
    gearing: Double,
    moi: MomentOfInertia,
    armLength: Distance
): ClimbIO {

    private val armSim: SingleJointedArmSim = SingleJointedArmSim(
        motor,
        gearing,
        moi into KilogramSquareMeters,
        armLength into Meters,
        Units.degreesToRadians(-45.0),
        Units.degreesToRadians(180.0 + 45.0),
        true,
        PI / 2
    )

    private val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()

    override fun updateInputs(inputs: ClimbIO.ClimbArmInputs) {

        armSim.update(0.02)

        inputs.motorVoltage.mut_replace(appliedVoltage)
        inputs.supplyVoltage.mut_replace(Volts.of(12.0))
        inputs.statorCurrent.mut_replace(armSim.currentDrawAmps, Amps)
        inputs.angularVelocity.mut_replace(armSim.velocityRadPerSec, RadiansPerSecond)
        inputs.absoluteAngle.mut_replace(armSim.angleRads, Radians)
    }

    override fun setVoltage(voltage: Voltage) {
        appliedVoltage.mut_replace(voltage)
        armSim.setInputVoltage(voltage into Volts)
    }

    override fun stop() {
        appliedVoltage.mut_replace(Volts.zero())
        armSim.setInputVoltage(0.0)
    }
}