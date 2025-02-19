package frc.robot.subsystems.superstructure

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.subsystems.superstructure.SuperstructureGoals.SuperstructureStateStruct
import frc.robot.subsystems.swerve.DriveLimits
import java.util.*

data class SuperstructureState(
    val armAngle: Angle,
    val elevatorHeight: Distance,
    val driveLimits: DriveLimits,
    val nextState: Optional<SuperstructureState> = Optional.empty()
) : StructSerializable {
    companion object {
        val struct = SuperstructureStateStruct()
    }
}