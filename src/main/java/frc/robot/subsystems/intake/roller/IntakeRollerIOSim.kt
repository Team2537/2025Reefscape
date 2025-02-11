package frc.robot.subsystems.intake.roller

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.math.units.into

class IntakeRollerIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val moi: MomentOfInertia,
): IntakeRollerIO {
    private val rollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(motor, moi into KilogramSquareMeters, gearing),
        motor
    )

    override fun updateInputs(inputs: IntakeRollerIO.IntakeRollerInputs) {
        rollerSim.update(0.02)

        inputs.position.mut_replace(rollerSim.angularPosition)
        inputs.velocity.mut_replace(rollerSim.angularVelocity)
        inputs.statorCurrent.mut_replace(rollerSim.currentDrawAmps, Amps)
        inputs.appliedVoltage.mut_replace(rollerSim.inputVoltage, Volts)
    }

    override fun setVoltage(voltage: Voltage) {
        rollerSim.inputVoltage = voltage into Volts
    }
}