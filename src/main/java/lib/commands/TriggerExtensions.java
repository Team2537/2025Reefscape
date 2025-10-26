package lib.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Simple helpers to mirror the former Kotlin trigger extensions. */
public final class TriggerExtensions {
  private TriggerExtensions() {}

  public static Trigger negate(Trigger trigger) {
    return trigger.negate();
  }
}
