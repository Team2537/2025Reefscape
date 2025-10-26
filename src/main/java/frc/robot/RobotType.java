package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Robot hardware/mode description helpers. */
public final class RobotType {
  private RobotType() {}

  public enum Type {
    ROBOT_2025_COMP,
    ROBOT_2025_SWERVE_BASE
  }

  public enum Mode {
    REAL(Type.ROBOT_2025_COMP),
    SIMULATION(Type.ROBOT_2025_COMP),
    REPLAY(Type.ROBOT_2025_COMP);

    private final Type defaultRobotType;

    Mode(Type defaultRobotType) {
      this.defaultRobotType = defaultRobotType;
    }

    public Type getDefaultRobotType() {
      return defaultRobotType;
    }
  }

  private static final boolean REPLAY_ENABLED = false;

  public static final Mode MODE =
      RobotBase.isReal() ? Mode.REAL : (REPLAY_ENABLED ? Mode.REPLAY : Mode.SIMULATION);

  public static final Type TYPE = MODE.getDefaultRobotType();

  public static final boolean IS_TUNING = !DriverStation.isFMSAttached();
}
