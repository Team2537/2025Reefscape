package lib.commands

import edu.wpi.first.wpilibj2.command.button.Trigger

operator fun Trigger.not(): Trigger {
    return this.negate()
}
