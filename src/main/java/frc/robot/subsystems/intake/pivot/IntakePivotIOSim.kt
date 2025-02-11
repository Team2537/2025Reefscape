package frc.robot.subsystems.intake.pivot

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.into

class IntakePivotIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val moi: MomentOfInertia,
    private val pidGains: PIDGains,
    private val ffGains: FeedforwardGains,
    private val kG: Double,
) : IntakePivotIO {
    private val pivotSim: SingleJointedArmSim =
        SingleJointedArmSim(
            motor,
            gearing,
            moi into KilogramSquareMeters,
            armLength into Meters,
            minAngle into Radians,
            maxAngle into Radians,
            true,
            startAngle into Radians,
        )

    private val controller: PIDController =
        PIDController(pidGains.kP, pidGains.kI, pidGains.kD)

    private val feedforward: ArmFeedforward =
        ArmFeedforward(ffGains.kV, kG, ffGains.kS, ffGains.kA)

    private val appliedVoltage: MutVoltage = Volts.zero().mutableCopy()
    private var isClosedLoop = false

    override fun updateInputs(inputs: IntakePivotIO.IntakePivotInputs) {
        pivotSim.update(0.02)

        inputs.position.mut_replace(pivotSim.angleRads, Radians)
        inputs.velocity.mut_replace(pivotSim.velocityRadPerSec, RadiansPerSecond)
        inputs.appliedVoltage.mut_replace(appliedVoltage)

        if(isClosedLoop) {
            val output = controller.calculate(pivotSim.angleRads, 0.0)
            val feedforwardOutput = feedforward.calculate(pivotSim.angleRads, 0.0)
            pivotSim.setInputVoltage(output + feedforwardOutput)
            appliedVoltage.mut_replace(output + feedforwardOutput, Volts)
        }
    }

    override fun setVoltage(volts: Voltage) {
        appliedVoltage.mut_replace(volts)
        pivotSim.setInput(volts into Volts)
        isClosedLoop = false
    }

    override fun setTargetAngle(angle: Angle) {
        isClosedLoop = true
        controller.setpoint = angle into Rotations
    }

    companion object {
        val armLength = Inches.of(20.0)
        val maxAngle = Degrees.of(90.0)
        val minAngle = Degrees.of(45.0)
        val startAngle = Degrees.of(90.0)
    }
}