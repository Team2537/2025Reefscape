package frc.robot.subsystems.arm

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.MomentOfInertiaUnit
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import frc.robot.subsystems.Superstructure
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.volts
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Arm: SubsystemBase("arm") {
    val io: ArmIO = when(RobotType.mode){
        RobotType.Mode.SIMULATION -> ArmIOSim(
            motor = DCMotor.getKrakenX60Foc(1),
            gearing = 50.0,
            moi = KilogramSquareMeters.of(0.164586918),
            pidGains = PIDGains(kP = 100.0),
            ffGains = FeedforwardGains(kV = 0.99),
            kG = 0.21,
        )
        else -> object : ArmIO {}
    }
    
    val inputs: ArmIO.ArmInputs = ArmIO.ArmInputs()
    
    
    fun getSendToAngleCmd(angle: Angle): Command {
        return runOnce { io.setAngle(angle) }
    }
    
    fun getSendToNodeCmd(setpoint: Superstructure.SuperstructureSetpoint): Command {
        return runOnce { io.setAngle(setpoint.armAngle) }
    }
    
    fun getManualMoveCmd(voltage: DoubleSupplier): Command {
        return run { io.setVoltage(voltage.asDouble.volts) }.handleInterrupt { io.setAngle(inputs.motorAbsolutePosition) }
    }
    
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("superstructure/$name", inputs)
        
        MechanismVisualizer.setArmAngle(inputs.motorRelativePosition)
    }
}