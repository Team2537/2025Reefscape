package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.subsystems.elevator.Elevator
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.max
import kotlin.math.min

object MechanismVisualizer {
    /**
     * The poses of the mechanisms in the robot's coordinate system.
     * 0: Elevator Stage 1
     * 1: Elevator Carriage (Stage 2)
     * 2: Arm
     */
    val mechanismPoses: MutableList<Pose3d> = mutableListOf(
        Pose3d(),
        Pose3d(),
        Pose3d()
    )
    
    private var carriageHeight = 0.0
    
    fun setElevatorHeight(height: Distance) {
        val heightMeters = (height into Meters).coerceIn(0.0, Elevator.maxHeight into Meters)
        carriageHeight = heightMeters
        
        mechanismPoses[0] = Pose3d(
            Translation3d(0.0, 0.0, heightMeters / 2.0), Rotation3d()
        )
        mechanismPoses[1] = Pose3d(
            Translation3d(0.0, 0.0, heightMeters), Rotation3d()
        )
    }
    
    fun setArmAngle(angle: Angle) {
        val angleRadians = angle into Radians
        
        mechanismPoses[2] = Pose3d(
            Translation3d(
                0.095880,
                -0.044220,
                0.197480 + carriageHeight
            ), Rotation3d(0.0, -angleRadians, 0.0)
        )
    }
    
    fun updatePoses() {
        Logger.recordOutput("mechanismPoses", *mechanismPoses.toTypedArray())
    }
}