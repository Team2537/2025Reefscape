package frc.robot.subsystems.superstructure.arm

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.Constants.ArmConstants
import frc.robot.subsystems.superstructure.SuperstructureGoals
import lib.math.units.into

class ArmIOSim(
    gearing: Double = ArmConstants.GEAR_RATIO * ArmConstants.CHAIN_RATIO,
    moi: MomentOfInertia = KilogramSquareMeters.of(0.15),
    armLength: Distance = Meters.of(0.35),
    pid: Triple<Double, Double, Double> = Triple(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD),
    ffGains: Quadruple = Quadruple(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA)
) : ArmIO {

    data class Quadruple(val kS: Double, val kG: Double, val kV: Double, val kA: Double)

    private val sim: SingleJointedArmSim = SingleJointedArmSim(
        DCMotor.getNEO(2),
        gearing,
        moi into KilogramSquareMeters,
        armLength into Meters,
        ArmConstants.MIN_ANGLE into Radians,
        ArmConstants.MAX_ANGLE into Radians,
        true,
        SuperstructureGoals.STOW.armAngle into Radians,
    )

    private val controller = PIDController(pid.first, pid.second, pid.third)
    private val feedforward = ArmFeedforward(ffGains.kS, ffGains.kG, ffGains.kV, ffGains.kA)

    private var positionControl: Boolean = false
    private var targetAngle: Angle = Radians.of(0.0)
    private var appliedVoltage: Voltage = Volts.zero()

    override fun updateInputs(inputs: ArmIO.ArmInputs) {
        if (positionControl) {
            val pidOut = controller.calculate(sim.angleRads, targetAngle into Radians)
            val ffOut = feedforward.calculate(targetAngle into Radians, 0.0)
            appliedVoltage = Volts.of(pidOut + ffOut)
        }

        sim.setInputVoltage(appliedVoltage into Volts)
        sim.update(0.02)

        inputs.isLeftMotorConnected = true
        inputs.isRightMotorConnected = true
        inputs.angle.mut_replace(sim.angleRads, Radians)
        inputs.angularVelocity.mut_replace(sim.velocityRadPerSec, RadiansPerSecond)
        inputs.appliedVoltage.mut_replace(appliedVoltage)
        inputs.leftStatorCurrent.mut_replace(Amps.of(sim.currentDrawAmps))
        inputs.rightStatorCurrent.mut_replace(Amps.of(sim.currentDrawAmps))
    }

    override fun setVoltage(voltage: Voltage) {
        positionControl = false
        appliedVoltage = voltage
    }

    override fun setTargetAngle(angle: Angle) {
        positionControl = true
        targetAngle = angle
        controller.reset()
    }

    override fun setBrakeMode(brake: Boolean) { /* no-op for sim */ }

    override fun stop() { appliedVoltage = Volts.zero() }
}


