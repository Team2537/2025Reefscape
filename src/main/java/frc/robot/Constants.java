package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
  private Constants() {
  }

  public static final class ArmConstants {
    public static final int LEFT_MOTOR_ID = 22;
    public static final int RIGHT_MOTOR_ID = 23;

    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = false;

    public static final double GEAR_RATIO = 20.0;
    public static final double CHAIN_RATIO = 4.0;

    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(170.0);
    public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(90.0);

    public static final double KP = 50.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double KS = 0.0;
    public static final double KG = 0.2;
    public static final double KV = 9.93;
    public static final double KA = 0.0;

    private ArmConstants() {
    }
  }

  public static final class ManipulatorConstants {
    public static final int LEFT_ROLLER_MOTOR_ID = 50;
    public static final int RIGHT_ROLLER_MOTOR_ID = 51;
    public static final int CANANDCOLOR_ID = 5;
    public static final boolean LEFT_ROLLER_INVERTED = true;
    public static final boolean RIGHT_ROLLER_INVERTED = true;
    public static final double ROLLER_GEARING = 1.0;

    public static final Voltage ROLLER_IN_VOLTAGE = Units.Volts.of(3.0);
    public static final Voltage ROLLER_OUT_VOLTAGE = Units.Volts.of(-4.0);
    public static final Current ROLLER_IN_TORQUE_CURRENT = Units.Amps.of(10.0);
    public static final Current ROLLER_OUT_TORQUE_CURRENT = Units.Amps.of(-30.0);

    public static final Distance DETECTION_DISTANCE_THRESHOLD = Units.Meters.of(0.1);

    public static final MomentOfInertia SIM_ROLLER_MOI = Units.KilogramSquareMeters.of(0.1);

    private ManipulatorConstants() {
    }
  }
}
