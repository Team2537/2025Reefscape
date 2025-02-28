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
        val CLIMB_MOTOR_ID: Int = 1
        val IS_CLIMB_MOTOR_INVERTED: Boolean = true
        val CLIMB_GEAR_RATIO: Double = 1.0
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
        val maxHeight = 1.302 measuredIn Meters
    }

    object GripperConstants {

    }

    object IntakeConstants {
        object PivotConstants {

        }

        object RollerConstants {
            val MOTOR_ID: Int = 0 // TODO: Set actual motor ID
            val IS_MOTOR_INVERTED: Boolean = false
            val GEARING: Double = 1.0 // TODO: Set actual gearing ratio
            val MOI: MomentOfInertia = KilogramSquareMeters.of(0.0) // TODO: Set actual moment of inertia
            val ROLLER_VOLTAGE: Voltage = Volts.of(12.0) // TODO: Set actual voltage
            val MINIMUM_ROLLER_VELOCITY: AngularVelocity = RotationsPerSecond.of(5.0) // TODO: Set actual minimum velocity
        }
    }

    // still may need to add constants for other parts of the robot
}
