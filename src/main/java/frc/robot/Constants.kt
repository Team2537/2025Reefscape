package frc.robot

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.units.measure.AngularVelocity

object Constants {


    object ArmConstants {
    
    }

    object ClimbConstants {

    }

    object ElevatorConstants {

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
