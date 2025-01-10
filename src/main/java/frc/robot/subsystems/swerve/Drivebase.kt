package frc.robot.subsystems.swerve

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d
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
    val moduleTranslations: Array<Translation2d> = when(RobotType.type){
        else -> arrayOf(
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
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[0]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[1]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[2]),
        SwerveModule(0, 0, 0, false, false, Rotation2d(), moduleTranslations[3])
    )
    
    val gyro: GyroIO = when(RobotType.mode){
        RobotType.Mode.SIMULATION -> GyroIOSim(::chassisSpeeds)
        else -> object : GyroIO {}
    }
    
    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*moduleTranslations)

    val odometry: SwerveDrivePoseEstimator3d = SwerveDrivePoseEstimator3d(
        kinematics,
        gyroInputs.fullRotation,
        wheelPositions,
        Pose3d()
    )

    val wheelPositions: Array<SwerveModulePosition>
        get() = modules.map { it.position }.toTypedArray()

    val wheelStates: Array<SwerveModuleState>
        get() = modules.map { it.state }.toTypedArray()

    val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*wheelStates)

    val pose: Pose3d
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
                    pose.rotation.toRotation2d()
                )
            } else {
                speeds = ChassisSpeeds(forwardS, strafeS, rotationS)
            }

            applyChassisSpeeds(speeds)
            println(speeds)
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
            gyroInputs.fullRotation,
            wheelPositions,
        )

        Logger.recordOutput("$name/pose", Pose3d.struct, pose)
        Logger.recordOutput("$name/chassisSpeeds", chassisSpeeds)
        Logger.recordOutput("$name/wheelStates", *wheelStates)
    }
    
    companion object Constants {
        val maxSpeed = 15.0 measuredIn FeetPerSecond
    }
}