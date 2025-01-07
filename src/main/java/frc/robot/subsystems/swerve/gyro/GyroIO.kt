package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.MutAngularVelocity
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    class GyroInputs : LoggableInputs {
        var yaw: Rotation2d = Rotation2d()
        var pitch: Rotation2d = Rotation2d()
        var roll: Rotation2d = Rotation2d()
        var yawRate: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var pitchRate: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var rollRate: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        
        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("yaw", yaw)
            table.put("pitch", pitch)
            table.put("roll", roll)
            table.put("yawRate", yawRate)
            table.put("pitchRate", pitchRate)
            table.put("rollRate", rollRate)
        }
        
        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            yaw = table.get("yaw", Rotation2d.struct, yaw)
            pitch = table.get("pitch", Rotation2d.struct, pitch)
            roll = table.get("roll", Rotation2d.struct, roll)
            
            yawRate.mut_replace(table.get("yawRate", yawRate))
            pitchRate.mut_replace(table.get("pitchRate", pitchRate))
            rollRate.mut_replace(table.get("rollRate", rollRate))
        }
    }
}