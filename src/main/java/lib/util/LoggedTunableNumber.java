package lib.util;

import frc.robot.RobotType;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Tunable number wrapper that only activates when the robot is in tuning mode. */
public final class LoggedTunableNumber implements DoubleSupplier {
  private final String key;
  private final double defaultValue;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();
  private final LoggedNetworkNumber dashboardNumber;

  public LoggedTunableNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.dashboardNumber = RobotType.IS_TUNING ? new LoggedNetworkNumber(key, defaultValue) : null;
  }

  @Override
  public double getAsDouble() {
    if (RobotType.IS_TUNING && dashboardNumber != null) {
      return dashboardNumber.get();
    }
    return defaultValue;
  }

  public boolean hasChanged(int id) {
    double currentValue = getAsDouble();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || Double.compare(currentValue, lastValue) != 0) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... numbers) {
    if (numbers == null || numbers.length == 0) {
      return;
    }
    for (LoggedTunableNumber number : numbers) {
      if (number.hasChanged(id)) {
        action.run();
        return;
      }
    }
  }

  public static void ifChangedValues(
      int id, java.util.function.Consumer<List<Double>> action, LoggedTunableNumber... numbers) {
    if (numbers == null || numbers.length == 0) {
      return;
    }
    boolean changed = false;
    for (LoggedTunableNumber number : numbers) {
      if (number.hasChanged(id)) {
        changed = true;
      }
    }
    if (changed) {
      action.accept(
          Arrays.stream(numbers)
              .map(LoggedTunableNumber::getAsDouble)
              .toList());
    }
  }
}
