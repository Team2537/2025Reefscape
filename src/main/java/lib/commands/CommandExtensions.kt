package lib.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import java.util.function.BooleanSupplier

fun Command.withIsFinished(isFinished: BooleanSupplier): WrapperCommand {
    return object : WrapperCommand(this) {
        override fun isFinished(): Boolean {
            return isFinished.asBoolean
        }
    }
}