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
        const val MOTOR_ID = 22
        const val IS_MOTOR_INVERTED = true
        const val GEARING = 40.0
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

        val PIVOT_DEPLOYED_ANGLE: Angle = Degrees.of(0.0) // TODO: Set actual angles
        val PIVOT_RETRACTED_ANGLE: Angle = Degrees.of(130.0)
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

        val REAL_PIVOT_KP = 30.0
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
        val PIVOT_UP_MAX_ANGLE: Angle = Degrees.of(0.0) // TODO: Set actual angles
        val PIVOT_DOWN_MAX_ANGLE: Angle = Degrees.of(130.0)
        val PIVOT_START_ANGLE: Angle = Degrees.of(0.0) // TODO: Set actual angle
        val ROLLER_IN_VOLTAGE: Voltage = Volts.of(3.0) // TODO: determine which sign is in and out, and set value
        val ROLLER_OUT_VOLTAGE: Voltage = Volts.of(-3.0)

        val PIVOT_MOTOR_ID = 4 // TODO: set actual values here
        val CANANDCOLOR_ID = 5
        val ROLLER_MOTOR_ID = 6
        val PIVOT_INVERTED = false
        val ROLLER_INVERTED = true
        val PIVOT_GEARING = 1.0
        val ROLLER_GEARING = 1.0

        val REAL_PIVOT_KP = 10.0
        val REAL_PIVOT_KV = 0.0
        val REAL_PIVOT_KA = 0.0
        val REAL_PIVOT_KG = 0.0
        val REAL_PIVOT_VELOCITY_LIMIT = RotationsPerSecond.of(10.0)
        val REAL_PIVOT_ACCELERATION_LIMIT = RotationsPerSecondPerSecond.of(10.0)
        val REAL_PIVOT_JERK_LIMIT = RotationsPerSecondPerSecond.per(Second).of(10.0)

        val SIM_PIVOT_MOI = KilogramSquareMeters.of(0.09)
        val SIM_ROLLER_MOI = KilogramSquareMeters.of(0.01)
        val SIM_PIVOT_KP = 10.0
        val SIM_PIVOT_KV = 0.0
        val SIM_PIVOT_KA = 0.0
        val SIM_PIVOT_KG = 0.0

        // see also SuperstructureGoals for pivot angle values for different levels
    }

    object TransferConstants {
        val MOTOR_ID = 0 // TODO: set actual values here
        val IS_MOTOR_INVERTED = false
        val GEARING = 1.0
        val MOI = KilogramSquareMeters.of(0.0)
        val TRANSFER_VOLTAGE = Volts.of(4.0)
        val MINIMUM_TRANSFER_VELOCITY = RotationsPerSecond.of(5.0)
    }

    // still may need to add constants for other parts of the robot
}
