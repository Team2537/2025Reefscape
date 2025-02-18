package frc.robot.subsystems.superstructure.arm

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import frc.robot.subsystems.superstructure.Superstructure
import lib.controllers.gains.FeedforwardGains
import lib.controllers.gains.PIDGains
import lib.math.units.epsilonEquals
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
    
    val setpoint: MutAngle = inputs.motorRelativePosition.mutableCopy()
    
    val positionInTolerance: Trigger = Trigger {
        inputs.motorRelativePosition.epsilonEquals(setpoint, Units.degreesToRadians(5.0))
    }
    
    
    fun getSendToAngleCmd(angle: Angle): Command {
        return runOnce {
            setpoint.mut_replace(angle)
            io.setAngle(angle)
        }
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