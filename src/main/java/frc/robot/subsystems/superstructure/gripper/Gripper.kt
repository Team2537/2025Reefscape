package frc.robot.subsystems.superstructure.gripper

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotType
import lib.math.units.volts
import org.littletonrobotics.junction.Logger

class Gripper: SubsystemBase("gripper") {
    val io: GripperIO = when(RobotType.mode){
        RobotType.Mode.SIMULATION -> GripperIOSim(
            DCMotor.getNEO(1),
            1.0,
            KilogramSquareMeters.of(0.001199574)
        )
        RobotType.Mode.REAL -> GripperIONeo(
            id = 31,
            sensorID = 0,
            isInverted = false,
            gearing = 4.0
        )
        else -> object : GripperIO {}
    }
    
    val inputs: GripperIO.GripperInputs = GripperIO.GripperInputs()
    
    val isHoldingCoral = Trigger { inputs.isHoldingCoral }
    
    fun getIntakeCmd(): Command = Commands.sequence(
        runOnce { io.setVoltage((-3.0).volts) },
        Commands.either(
            Commands.waitUntil { inputs.isHoldingCoral },
            Commands.waitSeconds(0.75),
            { RobotBase.isReal() }
        ),
        Commands.waitSeconds(0.05),
        runOnce { io.setVoltage(Volts.zero()) }
    ).handleInterrupt { io.setVoltage(Volts.zero())}
    
    fun getEjectCmd(): Command = Commands.sequence(
        runOnce { io.setVoltage(9.0.volts) },
        Commands.either(
            Commands.waitUntil { !inputs.isHoldingCoral },
            Commands.waitSeconds(0.75),
            { RobotBase.isReal() }
        ),
        runOnce { io.setVoltage(0.0.volts) }
    )
    
    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("superstructure/$name", inputs)
    }
}