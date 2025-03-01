package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.intake.pivot.IntakePivot
import frc.robot.subsystems.intake.roller.IntakeRoller
import org.littletonrobotics.junction.Logger

class Intake {
    val pivot = IntakePivot()
    val roller = IntakeRoller()

    private var isHoldingAlgae = false

    val isHoldingAlgaeTrig = Trigger { isHoldingAlgae }

    fun getIntakeCommand() =
        Commands.sequence(
            pivot.getSlamDownCommand(),
            roller.getSuckCommand(),
            Commands.print("Intake Command")
        ).andThen(
            pivot.getSlamUpCommand(),
            Commands.runOnce({ isHoldingAlgae = true })
        )

    fun getEjectCommand() =
        Commands.sequence(
            pivot.getSlamDownCommand(),
            roller.getSpitCommand(),
            Commands.print("Eject Command")
        ).andThen(
            pivot.getSlamUpCommand(),
            Commands.runOnce({ isHoldingAlgae = false })
        )

    fun periodic() {
        Logger.recordOutput("intake/isHoldingAlgae", isHoldingAlgae)
    }

}