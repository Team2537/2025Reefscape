package frc.robot.subsystems.superstructure.manipulator.roller

import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotType
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import lib.math.units.amps
import lib.math.units.rotationsPerSecond
import org.littletonrobotics.junction.Logger

class ManipulatorRoller: SubsystemBase("roller") {
    val io: ManipulatorRollerIO = when(RobotType.mode) {
        RobotType.Mode.REAL -> ManipulatorRollerIOKraken(
            motorID = 25,
            sensorID = 0,
            PIDGains(),
            FeedforwardGains(),
            22.0/18.0,
            Inches.of(1.5)
        )
        else -> object : ManipulatorRollerIO {}
    }

    val inputs: ManipulatorRollerIO.ManipulatorRollerInputs = ManipulatorRollerIO.ManipulatorRollerInputs()

    fun getIntakeCommand(): Command {
        return Commands.sequence(
            runOnce { io.setVoltage(Volts.of(4.0)) },
            Commands.waitUntil { inputs.coralDistance < 0.5 },
            runOnce { io.setVoltage(Volts.zero() )}
        )
    }

    fun getScoreCommand(): Command {
        return Commands.sequence(
            runOnce { io.setVoltage(Volts.of(3.0)) },
            Commands.waitUntil({ inputs.coralDistance > 0.2 }).withTimeout(0.4),
            runOnce { io.setVoltage(Volts.zero() )}
        )
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("manipulator/$name", inputs)
    }

    fun getDealgaefyCommand(): Command {
        return Commands.sequence(
            runOnce { io.setVoltage(Volts.of(-4.0)) },
            Commands.waitUntil {
                inputs.statorCurrent >= 30.0.amps
                        && inputs.angularVelocity < 0.5.rotationsPerSecond
            },
            runOnce { io.setVoltage(Volts.zero())}
        ).handleInterrupt { io.setVoltage(Volts.zero())}
    }
}