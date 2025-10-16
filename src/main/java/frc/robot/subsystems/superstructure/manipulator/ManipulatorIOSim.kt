package frc.robot.subsystems.superstructure.manipulator

import frc.robot.Constants
import frc.robot.subsystems.superstructure.manipulator.ManipulatorIO
import edu.wpi.first.units.measure.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import lib.math.units.into
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.measure.MomentOfInertia

class ManipulatorIOSim(
    rollerGearing: Double = 1.0,
    rollerMoi: MomentOfInertia = KilogramSquareMeters.of(0.01),
) : ManipulatorIO {
    // Two roller simulations
    private val leftRollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            rollerMoi into KilogramSquareMeters,
            rollerGearing
        ),
        DCMotor.getKrakenX60(1)
    )

    private val rightRollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            rollerMoi into KilogramSquareMeters,
            rollerGearing
        ),
        DCMotor.getKrakenX60(1)
    )
        
    override fun updateInputs(inputs: ManipulatorIO.ManipulatorInputs) {
        leftRollerSim.update(0.02)
        rightRollerSim.update(0.02)

        // Emulate Brake neutral mode: if commanded 0 V, clamp velocity to 0
        if (kotlin.math.abs(leftRollerSim.inputVoltage) < 1e-6) {
            leftRollerSim.setState(leftRollerSim.angularPositionRad, 0.0)
        }
        if (kotlin.math.abs(rightRollerSim.inputVoltage) < 1e-6) {
            rightRollerSim.setState(rightRollerSim.angularPositionRad, 0.0)
        }

        inputs.leftRollerAngularVelocity.mut_replace(leftRollerSim.angularVelocity)
        inputs.leftRollerAppliedVoltage.mut_replace(Volts.of(leftRollerSim.inputVoltage))
        inputs.leftRollerStatorCurrent.mut_replace(Amps.of(leftRollerSim.currentDrawAmps))

        inputs.rightRollerAngularVelocity.mut_replace(rightRollerSim.angularVelocity)
        inputs.rightRollerAppliedVoltage.mut_replace(Volts.of(rightRollerSim.inputVoltage))
        inputs.rightRollerStatorCurrent.mut_replace(Amps.of(rightRollerSim.currentDrawAmps))

        // Coral distance simulation (placeholder)
        inputs.coralDistance.mut_replace(Meters.of(0.5))
    }
    
    override fun setRollerVoltage(voltage: Voltage) {
        leftRollerSim.inputVoltage = voltage into Volts
        rightRollerSim.inputVoltage = voltage into Volts
        if (kotlin.math.abs(voltage into Volts) < 1e-6) {
            leftRollerSim.setState(leftRollerSim.angularPositionRad, 0.0)
            rightRollerSim.setState(rightRollerSim.angularPositionRad, 0.0)
        }
    }

    override fun setLeftRightRollerVoltages(leftVoltage: Voltage, rightVoltage: Voltage) {
        leftRollerSim.inputVoltage = leftVoltage into Volts
        rightRollerSim.inputVoltage = rightVoltage into Volts
        if (kotlin.math.abs(leftVoltage into Volts) < 1e-6) {
            leftRollerSim.setState(leftRollerSim.angularPositionRad, 0.0)
        }
        if (kotlin.math.abs(rightVoltage into Volts) < 1e-6) {
            rightRollerSim.setState(rightRollerSim.angularPositionRad, 0.0)
        }
    }

    override fun setRollerTorqueCurrent(current: Current) {
        // Approximate torque-current as proportional to motor torque -> acceleration; use as voltage proxy
        // For sim simplicity, scale current directly to input voltage range [-12, 12]
        val volts = (current into Amps) / 40.0 * 12.0
        setRollerVoltage(Volts.of(volts))
    }

    override fun setLeftRightRollerTorqueCurrents(leftCurrent: Current, rightCurrent: Current) {
        val leftVolts = (leftCurrent into Amps) / 40.0 * 12.0
        val rightVolts = (rightCurrent into Amps) / 40.0 * 12.0
        setLeftRightRollerVoltages(Volts.of(leftVolts), Volts.of(rightVolts))
    }

    override fun stopRoller() {
        leftRollerSim.inputVoltage = 0.0
        rightRollerSim.inputVoltage = 0.0
    }
} 