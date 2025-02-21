package frc.robot.subsystems.superstructure.gripper

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.math.units.into
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class GripperIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val moi: MomentOfInertia,
) : GripperIO {
    private val motorSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(motor, moi into KilogramSquareMeters, gearing),
        motor,
    )
    
    private val isHoldingCoral: LoggedNetworkBoolean = LoggedNetworkBoolean("/Tuning/gripper/isHoldingCoral", false)


    override fun updateInputs(inputs: GripperIO.GripperInputs) {
        motorSim.update(0.02)

        inputs.isHoldingCoral = isHoldingCoral.get()
        
        inputs.gripperPosition.mut_replace(motorSim.angularPosition)
        inputs.gripperVelocity.mut_replace(motorSim.angularVelocity)
        inputs.gripperAppliedVoltage.mut_replace(motorSim.inputVoltage, Volts)
        inputs.gripperStatorCurrent.mut_replace(motorSim.currentDrawAmps, Amps)
    }

    override fun setVoltage(voltage: Voltage) {
        motorSim.inputVoltage = voltage into Volts
    }
}