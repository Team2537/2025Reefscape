package frc.robot.subsystems.swerve

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.FeetPerSecond
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotType.Type.*
import frc.robot.RobotType
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.SwerveModule
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class Drivebase : SubsystemBase("drivebase") {
    
    /**
     * The translations of the modules relative to the center of the robot.
     *
     * This array contains the translations of the modules relative to the center of the robot. The
     * order of the modules is as follows:
     * 0: Front Left
     * 1: Front Right
     * 2: Back Left
     * 3: Back Right
     *
     * The translations are in meters.
     */
    val moduleTranslations: List<Translation2d> = when(RobotType.type){
        else -> listOf(
            Translation2d(Inches.of(10.0), Inches.of(10.0)),
            Translation2d(Inches.of(10.0), Inches.of(-10.0)),
            Translation2d(Inches.of(-10.0), Inches.of(10.0)),
            Translation2d(Inches.of(-10.0), Inches.of(-10.0))
        )
    }
    
    /**
     * The swerve modules on the robot.
     *
     * This array contains the swerve modules on the robot. The order of the modules is as follows:
     * 0: Front Left
     * 1: Front Right
     * 2: Back Left
     * 3: Back Right
     */
    val modules: Array<SwerveModule> = arrayOf(
        SwerveModule(1, 2, 3, false, false, Rotation2d.fromRadians(2.508), moduleTranslations[0]),
        SwerveModule(4, 5, 6, false, false, Rotation2d.fromRadians(1.155), moduleTranslations[1]),
        SwerveModule(7, 8, 9, false, false, Rotation2d.fromRadians(-1.982), moduleTranslations[2]),
        SwerveModule(10, 11, 12, false, false, Rotation2d.fromRadians(-1.764), moduleTranslations[3])
    )
    
    val gyro: GyroIO = when(RobotType.mode){
        RobotType.Mode.SIMULATION -> GyroIOSim(::chassisSpeeds)
        RobotType.Mode.REAL -> GyroIOPigeon2(13)
        else -> object : GyroIO {}
    }
    
    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*moduleTranslations.toTypedArray())

    val odometry: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyroInputs.yaw,
        wheelPositions.toTypedArray(),
        Pose2d()
    )

    val wheelPositions: List<SwerveModulePosition>
        get() = modules.map { it.position }

    val wheelStates: List<SwerveModuleState>
        get() = modules.map { it.state }

    val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*wheelStates.toTypedArray())

    val pose: Pose2d
        get() = odometry.estimatedPosition

    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        val discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val states = kinematics.toSwerveModuleStates(discretizedSpeeds)

        modules.zip(states).forEach { (module, state) -> module.applyState(state) }
    }

    fun getDriveCmd(
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        shouldFieldOrient: BooleanSupplier,
        slowmodeInput: DoubleSupplier
    ): Command {
        return run {
            var speeds = ChassisSpeeds()
            var forwardS = forward.asDouble
            var strafeS = strafe.asDouble
            var rotationS = rotation.asDouble

            // Potentially adjust the scaling factor to be a function of the slowmode input, rather than just multiplying by it
            forwardS *= slowmodeInput.asDouble
            strafeS *= slowmodeInput.asDouble

            if(shouldFieldOrient.asBoolean){
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardS,
                    strafeS,
                    rotationS,
                    pose.rotation
                )
            } else {
                speeds = ChassisSpeeds(forwardS, strafeS, rotationS)
            }

            applyChassisSpeeds(speeds)
        }
    }

    override fun periodic() {
        gyro.updateInputs(gyroInputs)
        modules.forEachIndexed { index, swerveModule ->
            swerveModule.periodic()
            Logger.processInputs("$name/modules/$index", swerveModule.inputs)
        }
        Logger.processInputs("$name/gyro", gyroInputs)

        odometry.update(
            gyroInputs.yaw,
            wheelPositions.toTypedArray(),
        )

        Logger.recordOutput("$name/pose", Pose2d.struct, pose)
        Logger.recordOutput("$name/chassisSpeeds", chassisSpeeds)
        Logger.recordOutput("$name/wheelStates", *wheelStates.toTypedArray())
    }
    
    companion object Constants {
        val maxSpeed = 15.0 measuredIn FeetPerSecond
    }
}