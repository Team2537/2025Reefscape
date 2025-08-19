package frc.robot.subsystems.intake

import frc.robot.subsystems.intake.IntakeIO
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

class IntakeIOSim(
    gearing: Double = 100.0,
    moi: MomentOfInertia = KilogramSquareMeters.of(0.1),
    pidGains: PIDGains = PIDGains(kP = 30.0),
    ffGains: FeedforwardGains = FeedforwardGains(kV = 0.0, kA = 0.0),
    kG: Double = 0.0,
) : IntakeIO {
    private val pivotSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            gearing,
            moi into KilogramSquareMeters,
            0.35, // arm length in meters (estimated from CAD)
            Radians.convertFrom(0.0, Degrees), // min angle in radians
            Radians.convertFrom(130.0, Degrees), // max angle in radians
            true,
            Radians.convertFrom(0.0, Degrees), // start angle in radians
        )

    // for pivot
    private val controller: PIDController =
        PIDController(pidGains.kP, pidGains.kI, pidGains.kD)

    private val feedforward: ArmFeedforward =
        ArmFeedforward(ffGains.kV, kG, ffGains.kS, ffGains.kA)
    
    // Roller simulation
    private val rollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), // Single Kraken X44 for roller, using Kraken X60 for simulation
            0.01, // Small MOI for roller
            1.0 // No gearing for roller
        ),
        DCMotor.getKrakenX60(1)
    )
    
    // Track applied voltages
    private var appliedPivotVoltage: Voltage = Volts.of(0.0)
    
    // Track target angle for position control
    private var targetAngle: Angle = Radians.of(0.0)
    private var usePositionControl: Boolean = false
        
    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        // Calculate control voltage if using position control
        if (usePositionControl) {
            val pidOutput = controller.calculate(pivotSim.angleRads, targetAngle into Radians)
            val ffOutput = feedforward.calculate(targetAngle into Radians, 0.0) // Assuming zero velocity setpoint
            appliedPivotVoltage = Volts.of(pidOutput + ffOutput)
        }
        
        // Update simulation with applied voltage
        pivotSim.setInputVoltage(appliedPivotVoltage into Volts)
        pivotSim.update(0.02)
        
        // Update roller simulation
        rollerSim.update(0.02)

        // Update inputs with simulation values - using correct field names from IntakeInputs
        inputs.pivotLeftPosition.mut_replace(pivotSim.angleRads, Radians)
        inputs.pivotLeftVelocity.mut_replace(pivotSim.velocityRadPerSec, RadiansPerSecond)
        inputs.pivotLeftAppliedVoltage.mut_replace(appliedPivotVoltage)
        inputs.pivotLeftStatorCurrent.mut_replace(Amps.of(pivotSim.currentDrawAmps))
        
        // Assume right motor follows left motor (as in real implementation)
        inputs.pivotRightPosition.mut_replace(pivotSim.angleRads, Radians)
        inputs.pivotRightVelocity.mut_replace(pivotSim.velocityRadPerSec, RadiansPerSecond)
        inputs.pivotRightAppliedVoltage.mut_replace(appliedPivotVoltage)
        inputs.pivotRightStatorCurrent.mut_replace(Amps.of(pivotSim.currentDrawAmps))
        
        // Update roller inputs using DCMotorSim
        inputs.rollerVelocity.mut_replace(rollerSim.angularVelocity)
        inputs.rollerAppliedVoltage.mut_replace(Volts.of(rollerSim.inputVoltage))
        inputs.rollerStatorCurrent.mut_replace(Amps.of(rollerSim.currentDrawAmps))
        
        // Coral distance simulation (placeholder)
        inputs.coralDistance.mut_replace(Meters.of(0.5))
    }

    override fun setPivotAngle(angle: Angle) {
        targetAngle = angle
        usePositionControl = true
        controller.reset()
    }

    override fun setPivotVoltage(voltage: Voltage) {
        appliedPivotVoltage = voltage
        usePositionControl = false
    }

    override fun setRollerVoltage(voltage: Voltage) {
        rollerSim.inputVoltage = voltage into Volts
    }

    override fun setPivotBrakeMode(brake: Boolean) {
        // In simulation, we don't need to do anything special for brake mode
        // The real implementation would change motor neutral mode
    }

    override fun stopRoller() {
        rollerSim.inputVoltage = 0.0
    }
}