package lib.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Command helpers replacing the Kotlin extension utilities. */
public final class CommandExtensions {
  private CommandExtensions() {}

  public static WrapperCommand withIsFinished(Command command, BooleanSupplier isFinished) {
    return new WrapperCommand(command) {
      @Override
      public boolean isFinished() {
        return isFinished.getAsBoolean();
      }
    };
  }
}
