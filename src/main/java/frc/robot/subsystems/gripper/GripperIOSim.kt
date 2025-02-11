package frc.robot.subsystems.gripper

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

class GripperIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains
) : GripperIO {
    private val motorSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(ffGains.kV, ffGains.kA),
        motor,
    )

    init {
        SmartDashboard.putBoolean("sim/gripper/isHoldingCoral", false)
    }

    override fun updateInputs(inputs: GripperIO.GripperInputs) {
        motorSim.update(0.02)

        inputs.isHoldingCoral = SmartDashboard.getBoolean("sim/gripper/isHoldingCoral", false)

        inputs.gripperPosition.mut_replace(motorSim.angularPosition)
        inputs.gripperVelocity.mut_replace(motorSim.angularVelocity)
        inputs.gripperAppliedVoltage.mut_replace(motorSim.inputVoltage, Volts)
        inputs.gripperStatorCurrent.mut_replace(motorSim.currentDrawAmps, Amps)
    }

    override fun setVoltage(voltage: Voltage) {
        motorSim.inputVoltage = voltage into Volts
    }
}