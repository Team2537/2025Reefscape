package frc.robot.subsystems.arm

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.math.units.into
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains

class ArmIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val moi: MomentOfInertia,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains,
    private val kG: Double
) : ArmIO {
    private val armSim: DCMotorSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor,
                moi into KilogramSquareMeters,
                gearing
            ),
            motor
        )

    private val feedforward: ArmFeedforward =
        ArmFeedforward(ffGains.kS, kG, ffGains.kV, ffGains.kA)

    private val controller: PIDController =
        PIDController(pidGains.kP, pidGains.kI, pidGains.kD)
            .apply { enableContinuousInput(0.0, 1.0) }

    private var isClosedLoop: Boolean = false

    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        armSim.update(0.02)

        inputs.motorVelocity.mut_replace(armSim.angularVelocity)
        inputs.motorAbsolutePosition.mut_replace(armSim.angularPosition)
        inputs.motorRelativePosition.mut_replace(armSim.angularPosition)
        inputs.motorSupplyVoltage.mut_replace(Volts.of(12.0))
        inputs.motorStatorCurrent.mut_replace(armSim.currentDrawAmps, Amps)
        inputs.motorVoltage.mut_replace(armSim.inputVoltage, Volts)

        if(isClosedLoop) {
            val voltage = controller.calculate(inputs.motorAbsolutePosition into Rotations)
            armSim.setInputVoltage(voltage + feedforward.calculate(controller.setpoint, 0.0))
        }
    }

    override fun setAngle(angle: Angle) {
        controller.setpoint = angle into Rotations
        isClosedLoop = true
    }

    override fun setVoltage(voltage: Voltage) {
        armSim.inputVoltage = voltage into Volts
        isClosedLoop = false
    }

    override fun stop() {
        armSim.inputVoltage = 0.0
        isClosedLoop = false
    }
}