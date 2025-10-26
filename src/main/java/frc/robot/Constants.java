package frc.robot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.angular.AngularAcceleration;
import edu.wpi.first.units.measure.angular.AngularVelocity;
import edu.wpi.first.units.measure.linear.LinearVelocity;

/**
 * Robot-wide numerical or unit-bearing constants. Converted from Kotlin object to Java class.
 */
public final class Constants {
  private Constants() {}

  public static final class ArmConstants {
    public static final int LEFT_MOTOR_ID = 22;
    public static final int RIGHT_MOTOR_ID = 23;

    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = false;

    public static final double GEAR_RATIO = 20.0;
    public static final double CHAIN_RATIO = 4.0;

    public static final Angle MIN_ANGLE = Units.Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Units.Degrees.of(170.0);
    public static final Angle STARTING_ANGLE = Units.Degrees.of(90.0);

    public static final double KP = 50.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double KS = 0.0;
    public static final double KG = 0.2;
    public static final double KV = 9.93;
    public static final double KA = 0.0;

    private ArmConstants() {}
  }

  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 23;
    public static final int THROUGHBORE_ID = 0;
    public static final boolean IS_CLIMB_MOTOR_INVERTED = true;
    public static final double CLIMB_GEAR_RATIO = 5600.0;
    public static final MomentOfInertia MOI = Units.KilogramSquareMeters.of(0.0);
    public static final Distance ARM_LENGTH = Units.Meters.of(0.0);

    public static final Angle RETRACTED_ANGLE = Units.Degrees.of(250.0);
    public static final Angle EXTENDED_ANGLE = Units.Degrees.of(0.0);
    public static final Angle ABSOLUTE_OFFSET = Units.Degrees.of(0.0);
    public static final Voltage EXTEND_VOLTAGE = Units.Volts.of(6.0);
    public static final Voltage RETRACT_VOLTAGE = Units.Volts.of(10.0);
    public static final Angle EXTEND_ANGLE_TOLERANCE = Units.Degrees.of(2.0);

    public static final Current CURRENT_SPIKE_THRESHOLD = Units.Amps.of(30.0);
    public static final AngularVelocity VELOCITY_ZERO_THRESHOLD = Units.RotationsPerSecond.of(0.1);

    private ClimbConstants() {}
  }

  public static final class ElevatorConstants {
    public static final Measure<Distance> MAX_HEIGHT = Units.Meters.of(1.9);
    public static final double GEAR_RATIO = 5.0;
    public static final Distance DRUM_RADIUS = Units.Inches.of(1.76);
    public static final int LEFT_MOTOR_ID = 21;
    public static final int RIGHT_MOTOR_ID = 20;
    public static final double KP = 10.0;
    public static final double KS = 0.082337;
    public static final double KV = 0.015551;
    public static final double KG = 0.44098;
    public static final LinearVelocity MOTION_MAGIC_CRUISE_VELOCITY = Units.MetersPerSecond.of(0.75);
    public static final LinearVelocity MOTION_MAGIC_ACCELERATION = Units.MetersPerSecondPerSecond.of(20.0);
    public static final double MOTION_MAGIC_JERK = 0.0;

    public static final Measure<Mass> SIM_MASS = Units.Pounds.of(10.0);
    public static final double SIM_KP = 10.0;
    public static final double SIM_KV = 1.77;
    public static final double SIM_KG = 0.19;

    private ElevatorConstants() {}
  }

  public static final class GripperConstants {
    public static final double CANANDCOLOR_INCH_SCALAR = 18.42105;

    private GripperConstants() {}
  }

  public static final class IntakeConstants {
    public static final Angle PIVOT_DEPLOYED_ANGLE = Units.Degrees.of(110.0);
    public static final Angle PIVOT_RETRACTED_ANGLE = Units.Degrees.of(0.0);
    public static final Voltage ROLLER_VOLTAGE = Units.Volts.of(3.0);

    public static final int LEFT_PIVOT_MOTOR_ID = 0;
    public static final int RIGHT_PIVOT_MOTOR_ID = 1;
    public static final int CANANDCOLOR_ID = 2;
    public static final int ROLLER_MOTOR_ID = 3;
    public static final boolean LEFT_PIVOT_INVERTED = false;
    public static final boolean RIGHT_PIVOT_INVERTED = true;
    public static final boolean ROLLER_INVERTED = true;
    public static final double PIVOT_GEARING = 1.0;
    public static final double ROLLER_GEARING = 1.0;

    public static final double REAL_PIVOT_KP = 5.0;
    public static final double REAL_PIVOT_KV = 0.0;
    public static final double REAL_PIVOT_KA = 0.0;
    public static final double REAL_PIVOT_KG = 0.0;
    public static final AngularVelocity REAL_PIVOT_VELOCITY_LIMIT = Units.RotationsPerSecond.of(10.0);
    public static final Measure<AngularAcceleration> REAL_PIVOT_ACCELERATION_LIMIT =
        Units.RotationsPerSecondPerSecond.of(10.0);
    public static final double REAL_PIVOT_JERK_LIMIT = 10.0;

    public static final MomentOfInertia SIM_MOI = Units.KilogramSquareMeters.of(0.09);
    public static final double SIM_PIVOT_KP = 30.0;
    public static final double SIM_PIVOT_KV = 0.0;
    public static final double SIM_PIVOT_KA = 0.0;
    public static final double SIM_PIVOT_KG = 0.0;

    private IntakeConstants() {}
  }

  public static final class ManipulatorConstants {
    public static final Distance PIVOT_ARM_LENGTH = Units.Meters.of(0.2);
    public static final Angle PIVOT_UP_MAX_ANGLE = Units.Degrees.of(-90.0);
    public static final Angle PIVOT_DOWN_MAX_ANGLE = Units.Degrees.of(130.0);
    public static final Angle PIVOT_START_ANGLE = Units.Degrees.of(90.0);
    public static final Voltage ROLLER_IN_VOLTAGE = Units.Volts.of(3.0);
    public static final Voltage ROLLER_OUT_VOLTAGE = Units.Volts.of(-4.0);
    public static final Current ROLLER_IN_TORQUE_CURRENT = Units.Amps.of(10.0);
    public static final Current ROLLER_OUT_TORQUE_CURRENT = Units.Amps.of(-30.0);

    public static final int LEFT_ROLLER_MOTOR_ID = 50;
    public static final int RIGHT_ROLLER_MOTOR_ID = 51;
    public static final int CANANDCOLOR_ID = 5;
    public static final boolean LEFT_ROLLER_INVERTED = true;
    public static final boolean RIGHT_ROLLER_INVERTED = true;
    public static final double ROLLER_GEARING = 1.0;

    public static final int PIVOT_MOTOR_ID = 4;
    public static final boolean PIVOT_INVERTED = false;
    public static final double PIVOT_GEARING = 1.0;
    public static final double REAL_PIVOT_KP = 10.0;
    public static final double REAL_PIVOT_KV = 0.0;
    public static final double REAL_PIVOT_KA = 0.0;
    public static final double REAL_PIVOT_KG = 0.0;
    public static final AngularVelocity REAL_PIVOT_VELOCITY_LIMIT = Units.RotationsPerSecond.of(10.0);
    public static final Measure<AngularAcceleration> REAL_PIVOT_ACCELERATION_LIMIT =
        Units.RotationsPerSecondPerSecond.of(10.0);
    public static final double REAL_PIVOT_JERK_LIMIT = 10.0;

    public static final MomentOfInertia SIM_PIVOT_MOI = Units.KilogramSquareMeters.of(0.09);
    public static final MomentOfInertia SIM_ROLLER_MOI = Units.KilogramSquareMeters.of(0.1);
    public static final double SIM_PIVOT_KP = 10.0;
    public static final double SIM_PIVOT_KI = 1.0;
    public static final double SIM_PIVOT_KD = 1.5;
    public static final double SIM_PIVOT_KV = 0.0;
    public static final double SIM_PIVOT_KA = 0.0;
    public static final double SIM_PIVOT_KG = 5.7;

    public static final Distance DETECTION_DISTANCE_THRESHOLD = Units.Meters.of(0.1);
    public static final Distance CANANDCOLOR_SCALING_FACTOR = Units.Meters.of(5.0);

    private ManipulatorConstants() {}
  }

  public static final class TransferConstants {
    public static final int MOTOR_ID = 0;
    public static final boolean IS_MOTOR_INVERTED = false;
    public static final double GEARING = 1.0;
    public static final MomentOfInertia MOI = Units.KilogramSquareMeters.of(0.05);
    public static final Voltage TRANSFER_VOLTAGE = Units.Volts.of(4.0);
    public static final AngularVelocity MINIMUM_TRANSFER_VELOCITY = Units.RotationsPerSecond.of(5.0);
    public static final Distance DETECTION_DISTANCE_THRESHOLD = Units.Meters.of(0.1);

    private TransferConstants() {}
  }
}
