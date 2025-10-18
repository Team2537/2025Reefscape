package frc.robot

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.MomentOfInertiaUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.units.measure.AngularVelocity
import lib.math.units.measuredIn

object Constants {

    object ArmConstants {
        // Motor CAN IDs (placeholder values)
        const val LEFT_MOTOR_ID = 22
        const val RIGHT_MOTOR_ID = 23

        // Inversions
        const val LEFT_INVERTED = true
        const val RIGHT_INVERTED = false

        const val GEAR_RATIO = 20.0
        const val CHAIN_RATIO = 4.0

        // Motion limits (mechanical hard limits) in degrees
        // Tune these to your actual arm travel
        val MIN_ANGLE = Degrees.of(0.0)
        val MAX_ANGLE = Degrees.of(170.0)
        
        // Starting angle to set encoders to on startup
        val STARTING_ANGLE = Degrees.of(90.0)

        // Control gains (initial guesses; tune)
        const val KP = 50.0
        const val KI = 0.0
        const val KD = 0.0

        // Arm feedforward (V = ks*sign + kg*cos(theta) + kv*vel + ka*acc)
        const val KS = 0.0
        const val KG = 0.2
        const val KV = 9.93
        const val KA = 0.0
    }

    object ClimbConstants {
        // PLACEHOLDER VALUES
        val CLIMB_MOTOR_ID: Int = 23
        val THROUGHBORE_ID: Int = 0
        val IS_CLIMB_MOTOR_INVERTED: Boolean = true
        val CLIMB_GEAR_RATIO: Double = 5600.0
        val MOI: MomentOfInertia = KilogramSquareMeters.of(0.0)
        val ARM_LENGTH: Distance = Meters.of(0.0)

        // 0 degrees is horizontal, with the end of the climb arm pointing out
        // positive angle is counterclockwise when viewed as if the climb side of the robot was on the right
        val RETRACTED_ANGLE: Angle = Degrees.of(250.0)
        val EXTENDED_ANGLE: Angle = Degrees.of(0.0) // horizontal, with arm pointing to the right
        val ABSOLUTE_OFFSET: Angle = Degrees.of(0.0) // offset of the absolute encoder from the zero position
        val EXTEND_VOLTAGE: Voltage = Volts.of(6.0)
        val RETRACT_VOLTAGE: Voltage = Volts.of(10.0)
        val EXTEND_ANGLE_TOLERANCE: Angle = Degrees.of(2.0)

        // Retraction detection thresholds
        val CURRENT_SPIKE_THRESHOLD = Amps.of(30.0)
        val VELOCITY_ZERO_THRESHOLD = RotationsPerSecond.of(0.1)
    }

    object ElevatorConstants {
        val maxHeight = 1.9 measuredIn Meters
        val GEAR_RATIO = 5.0
        val DRUM_RADIUS = Inches.of(1.76)
        val LEFT_MOTOR_ID = 21
        val RIGHT_MOTOR_ID = 20
        val KP = 10.0
        val KS = 0.082337
        val KV = 0.015551
        val KG = 0.44098
        val MOTION_MAGIC_CRUISE_VELOCITY = MetersPerSecond.of(0.75)
        val MOTION_MAGIC_ACCELERATION = MetersPerSecondPerSecond.of(20.0)
        val MOTION_MAGIC_JERK = 0.0

        val SIM_MASS = 10.0 measuredIn Pounds
        val SIM_KP = 10.0
        val SIM_KV = 1.77
        val SIM_KG = 0.19

        // see also SuperstructureGoals for elevator height values for different levels
    }

    object GripperConstants {
        const val CANANDCOLOR_INCH_SCALAR = 18.42105
    }

    object IntakeConstants {

        val PIVOT_DEPLOYED_ANGLE: Angle = Degrees.of(110.0)
        val PIVOT_RETRACTED_ANGLE: Angle = Degrees.of(0.0)
        val ROLLER_VOLTAGE: Voltage = Volts.of(3.0)

        val LEFT_PIVOT_MOTOR_ID = 0 // TODO: set actual values here
        val RIGHT_PIVOT_MOTOR_ID = 1
        val CANANDCOLOR_ID = 2
        val ROLLER_MOTOR_ID = 3
        val LEFT_PIVOT_INVERTED = false
        val RIGHT_PIVOT_INVERTED = true
        val ROLLER_INVERTED = true
        val PIVOT_GEARING = 1.0
        val ROLLER_GEARING = 1.0

        val REAL_PIVOT_KP = 5.0
        val REAL_PIVOT_KV = 0.0
        val REAL_PIVOT_KA = 0.0
        val REAL_PIVOT_KG = 0.0
        val REAL_PIVOT_VELOCITY_LIMIT = RotationsPerSecond.of(10.0)
        val REAL_PIVOT_ACCELERATION_LIMIT = RotationsPerSecondPerSecond.of(10.0)
        val REAL_PIVOT_JERK_LIMIT = RotationsPerSecondPerSecond.per(Second).of(10.0)

        val SIM_MOI = KilogramSquareMeters.of(0.09)
        val SIM_PIVOT_KP = 30.0
        val SIM_PIVOT_KV = 0.0
        val SIM_PIVOT_KA = 0.0
        val SIM_PIVOT_KG = 0.0
    }

    object ManipulatorConstants {
        val PIVOT_ARM_LENGTH: Distance = Meters.of(0.2)
        val PIVOT_UP_MAX_ANGLE: Angle = Degrees.of(-90.0)
        val PIVOT_DOWN_MAX_ANGLE: Angle = Degrees.of(130.0)
        val PIVOT_START_ANGLE: Angle = Degrees.of(90.0)
        val ROLLER_IN_VOLTAGE: Voltage = Volts.of(3.0) // TODO: determine which sign is in and out, and set value
        val ROLLER_OUT_VOLTAGE: Voltage = Volts.of(-12.0)
        // Torque-current control (Amps equivalent for FOC)
        val ROLLER_IN_TORQUE_CURRENT = Amps.of(10.0)
        val ROLLER_OUT_TORQUE_CURRENT = Amps.of(-30.0)

        // Dual roller configuration (no pivot)
        val LEFT_ROLLER_MOTOR_ID = 50 // TODO: set actual values here
        val RIGHT_ROLLER_MOTOR_ID = 51 // TODO: set actual values here
        val CANANDCOLOR_ID = 5
        val LEFT_ROLLER_INVERTED = true
        val RIGHT_ROLLER_INVERTED = true
        val ROLLER_GEARING = 1.0

        // Legacy pivot constants retained for compatibility elsewhere (unused by manipulator implementation)
        val PIVOT_MOTOR_ID = 4
        val PIVOT_INVERTED = false
        val PIVOT_GEARING = 1.0
        val REAL_PIVOT_KP = 10.0
        val REAL_PIVOT_KV = 0.0
        val REAL_PIVOT_KA = 0.0
        val REAL_PIVOT_KG = 0.0
        val REAL_PIVOT_VELOCITY_LIMIT = RotationsPerSecond.of(10.0)
        val REAL_PIVOT_ACCELERATION_LIMIT = RotationsPerSecondPerSecond.of(10.0)
        val REAL_PIVOT_JERK_LIMIT = RotationsPerSecondPerSecond.per(Second).of(10.0)

        val SIM_PIVOT_MOI = KilogramSquareMeters.of(0.09)
        val SIM_ROLLER_MOI = KilogramSquareMeters.of(0.1)
        val SIM_PIVOT_KP = 10.0
        val SIM_PIVOT_KI = 1.0
        val SIM_PIVOT_KD = 1.5
        val SIM_PIVOT_KV = 0.0
        val SIM_PIVOT_KA = 0.0
        val SIM_PIVOT_KG = 5.7

        val DETECTION_DISTANCE_THRESHOLD = Meters.of(0.1) // tune
        // represents distance when proximity is 1.0, or half of that when proximity is 0.5, etc.
        val CANANDCOLOR_SCALING_FACTOR = Meters.of(5.0) // tune

        // see also SuperstructureGoals for pivot angle values for different levels
    }

    object TransferConstants {
        val MOTOR_ID = 0 // TODO: set actual values here
        val IS_MOTOR_INVERTED = false
        val GEARING = 1.0
        val MOI = KilogramSquareMeters.of(0.05)
        val TRANSFER_VOLTAGE = Volts.of(4.0)
        val MINIMUM_TRANSFER_VELOCITY = RotationsPerSecond.of(5.0)

        val DETECTION_DISTANCE_THRESHOLD = Meters.of(0.1) // tune
    }

    // still may need to add constants for other parts of the robot
}
