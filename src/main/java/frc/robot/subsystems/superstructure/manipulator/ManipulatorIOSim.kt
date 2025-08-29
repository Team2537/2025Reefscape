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
    pivotGearing: Double = 100.0,
    rollerGearing: Double = 1.0,
    pivotMoi: MomentOfInertia = KilogramSquareMeters.of(0.1),
    rollerMoi: MomentOfInertia = KilogramSquareMeters.of(0.01),
    pivotPIDGains: PIDGains = PIDGains(kP = 30.0),
    pivotFFGains: FeedforwardGains = FeedforwardGains(kV = 0.0, kA = 0.0),
    pivotKG: Double = 0.0,
) : ManipulatorIO {
    private val pivotSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            pivotGearing,
            pivotMoi into KilogramSquareMeters,
            Constants.ManipulatorConstants.PIVOT_ARM_LENGTH into Meters, // arm length in meters (estimated from CAD)
            Constants.ManipulatorConstants.PIVOT_UP_MAX_ANGLE into Radians, // min angle in radians
            Constants.ManipulatorConstants.PIVOT_DOWN_MAX_ANGLE into Radians, // max angle in radians
            true,
            Constants.ManipulatorConstants.PIVOT_START_ANGLE into Radians, // start angle in radians
        )

    // for pivot
    private val controller: PIDController =
        PIDController(pivotPIDGains.kP, pivotPIDGains.kI, pivotPIDGains.kD) 

    private val feedforward: ArmFeedforward =
        ArmFeedforward(pivotFFGains.kV, pivotKG, pivotFFGains.kS, pivotFFGains.kA)
    
    // Roller simulation
    private val rollerSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), // Single Kraken X44 for roller, using Kraken X60 for simulation
            rollerMoi into KilogramSquareMeters, // Small MOI for roller
            rollerGearing // No gearing for roller
        ),
        DCMotor.getKrakenX60(1)
    )
    
    // Track applied voltages
    private var appliedPivotVoltage: Voltage = Volts.of(0.0)
    
    // Track target angle for position control
    private var targetAngle: Angle = Radians.of(0.0)
    private var usePositionControl: Boolean = false
        
    override fun updateInputs(inputs: ManipulatorIO.ManipulatorInputs) {
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

        // Update inputs with simulation values - using correct field names from ManipulatorInputs
        inputs.pivotAngularPosition.mut_replace(pivotSim.angleRads, Radians)
        inputs.pivotAngularVelocity.mut_replace(pivotSim.velocityRadPerSec, RadiansPerSecond)
        inputs.pivotAppliedVoltage.mut_replace(appliedPivotVoltage)
        inputs.pivotStatorCurrent.mut_replace(Amps.of(pivotSim.currentDrawAmps))
        inputs.pivotTargetAngularPosition.mut_replace(targetAngle)
        
        // Update roller inputs using DCMotorSim
        inputs.rollerAngularVelocity.mut_replace(rollerSim.angularVelocity)
        inputs.rollerAppliedVoltage.mut_replace(Volts.of(rollerSim.inputVoltage))
        inputs.rollerStatorCurrent.mut_replace(Amps.of(rollerSim.currentDrawAmps))
        
        // Coral distance simulation (placeholder)
        inputs.coralDistance.mut_replace(Meters.of(0.5))
    }
    
    override fun setPivotTargetAngle(angle: Angle) {
        targetAngle = angle
        usePositionControl = true
        controller.reset()
    }

    override fun setPivotVoltage(voltage: Voltage) {
        appliedPivotVoltage = voltage
        usePositionControl = false
    }

    // TODO: determine which rotational direction is positive
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