package frc.robot.subsystems.transfer

import frc.robot.subsystems.transfer.TransferIO
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.subsystems.transfer.TransferIO.TransferInputs
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MomentOfInertia
import lib.math.units.into


class TransferIOSim(
    private val gearing: Double,
    private val moi: MomentOfInertia,
) : TransferIO {
    private val rollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), // Single Kraken X44 for roller, using Kraken X60 for simulation
            moi into KilogramSquareMeters, // Small MOI for roller
            gearing // No gearing for roller
        ),
        DCMotor.getKrakenX60(1)
    )

    override fun updateInputs(inputs: TransferInputs) {
        rollerSim.update(0.02)

        inputs.velocity.mut_replace(rollerSim.angularVelocity)
        inputs.appliedVoltage.mut_replace(Volts.of(rollerSim.inputVoltage))
        inputs.statorCurrent.mut_replace(Amps.of(rollerSim.currentDrawAmps))
    }
    
    
}