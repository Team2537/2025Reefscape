package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.RobotType
import frc.robot.subsystems.superstructure.SuperstructureGoals
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.SwerveModule
import lib.math.units.into
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.*

class Drivebase : SubsystemBase("drivebase") {
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
        SwerveModule(1, 2, 3, true, true, Rotation2d.fromRadians(2.490), moduleTranslations[0]),
        SwerveModule(4, 5, 6, true, true, Rotation2d.fromRadians(1.2), moduleTranslations[1]),
        SwerveModule(7, 8, 9, true, true, Rotation2d.fromRadians(-2.008), moduleTranslations[2]),
        SwerveModule(10, 11, 12, true, true, Rotation2d(-1.640), moduleTranslations[3])
    )
    
    val gyro: GyroIO = when (RobotType.mode) {
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
        Pose2d(Translation2d(3.11, 4.04), Rotation2d()),
        VecBuilder.fill(0.0, 0.0, 0.0),
        VecBuilder.fill(0.9, 0.9, 0.9),
    )
    
    val wheelPositions: List<SwerveModulePosition>
        get() = modules.map { it.position }
    
    val wheelStates: List<SwerveModuleState>
        get() = modules.map { it.state }
    
    val desiredStates: List<SwerveModuleState>
        get() = modules.map { it.desiredState }
    
    val moduleForces: List<SwerveModuleState>
        get() = modules.map { it.wheelForce }
    
    val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*wheelStates.toTypedArray())
    
    val pose: Pose2d
        get() = Pose2d(odometry.estimatedPosition.translation, gyroInputs.yaw)
    
    val wheelRadiusCharacterizationAngles: List<Angle>
        get() = modules.map { it.radiusCharacterizationAngle }
    
    val driveSysIDVolts: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            2.0 measuredIn Seconds,
            { state ->
                Logger.recordOutput("$name/sysIdState", state.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            { volts: Voltage -> modules.map { it.characterizeDriveVoltage(volts) } },
            null,
            this
        )
    )
    
    val driveSysIDCurrent: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            2.0 measuredIn Seconds,
            { state ->
                Logger.recordOutput("$name/sysIdState", state.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            { current: Voltage -> modules.map { it.characterizeCurrent(current.baseUnitMagnitude() measuredIn Amps) } },
            null,
            this
        )
    )
    
    val steerSysIdRoutine: SysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                { state -> Logger.recordOutput("$name/state", state.toString()) }
            ),
            SysIdRoutine.Mechanism(
                { volts: Voltage -> modules.map { it.characterizeSteerVoltage(volts) } },
                null,
                this
            )
        )
    
    
    val routineToApply = steerSysIdRoutine
    
    var limits = defaultLimits
    
    val robotConfig: RobotConfig? = try {
        RobotConfig.fromGUISettings()
    } catch (e: Exception) {
        e.printStackTrace()
        null
    }
    
    init {
        AutoBuilder.configure(
            ::pose,
            ::resetOdometry,
            ::chassisSpeeds,
            { speeds: ChassisSpeeds, feedforward: DriveFeedforwards -> applyChassisSpeeds(speeds) },
            PPHolonomicDriveController(
                PIDConstants(3.0),
                PIDConstants(3.0),
            ),
            robotConfig,
            {
                DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
            },
            this
        )
    }
    
    fun applyChassisSpeeds(speeds: ChassisSpeeds, moduleForces: List<Vector<N2>>) {
        val discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        
        val states = kinematics.toSwerveModuleStates(discretizedSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            limits.maxLinVel,
        )
        
        modules.zip(states).forEachIndexed { index, (module, state) ->
            module.applyState(state, moduleForces[index])
        }
    }
    
    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        val discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        
        val states = kinematics.toSwerveModuleStates(discretizedSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            limits.maxLinVel,
        )
        
        modules.zip(states).forEach { (module, state) -> module.applyState(state) }
        Logger.recordOutput("$name/discretizedChassisSpeeds", discretizedSpeeds)
    }
    
    private fun applyLimits(speed: ChassisSpeeds): ChassisSpeeds {
        val currentSpeeds = chassisSpeeds
        
        // Current velocity magnitude
        val currentVelocityMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
        
        // Desired velocity magnitude
        val desiredVelocityMagnitude = hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond).coerceIn(
            0.0,
            limits.maxLinVel into MetersPerSecond
        )
        
        val desiredAccel = (desiredVelocityMagnitude - currentVelocityMagnitude) / 0.02 // Acceleration in m/s²
        
        
        val limitedAccel = desiredAccel.coerceIn(
            -limits.maxAccel into MetersPerSecondPerSecond,
            limits.maxAccel into MetersPerSecondPerSecond
        )
        
        // Calculate the new velocity magnitude with limited acceleration
        val newVelocityMagnitude = currentVelocityMagnitude + limitedAccel * 0.02 // New velocity in m/s
        
        // Scale the desired velocity vector to the new magnitude
        val scaleFactor = if (desiredVelocityMagnitude != 0.0) {
            newVelocityMagnitude / desiredVelocityMagnitude
        } else {
            0.0
        }
        
        val adjustedVx = speed.vxMetersPerSecond * scaleFactor
        val adjustedVy = speed.vyMetersPerSecond * scaleFactor
        
        return ChassisSpeeds(
            adjustedVx,
            adjustedVy,
            speed.omegaRadiansPerSecond.coerceIn(
                -limits.maxAngVel into RadiansPerSecond,
                limits.maxAngVel into RadiansPerSecond
            )
        )
    }
    
    
    fun runWheelRadiusCharacterization(omegaSpeed: AngularVelocity) {
        applyChassisSpeeds(
            ChassisSpeeds(
                MetersPerSecond.zero(), MetersPerSecond.zero(), omegaSpeed
            )
        )
    }
    
    fun getStopCmd() = runOnce {
        modules.zip(wheelStates).forEach { (module: SwerveModule, state: SwerveModuleState) ->
            module.applyState(SwerveModuleState(0.0, state.angle))
        }
    }
    
    fun getDriveCmd(
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        shouldFieldOrient: BooleanSupplier,
        slowmodeInput: DoubleSupplier,
        exponent: Int
    ): Command {
        return run {
            val speeds: ChassisSpeeds
            val magnitude = hypot(strafe.asDouble, forward.asDouble).pow(exponent)
            val direction = Rotation2d.fromRadians(atan2(forward.asDouble, strafe.asDouble))
            var rotationSpeed = rotation.asDouble
            
            val forwardS = magnitude * direction.sin
            val strafeS = magnitude * direction.cos
            
            
            if (shouldFieldOrient.asBoolean) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardS * (maxAttainableLinearVelocity into MetersPerSecond),
                    strafeS * (maxAttainableLinearVelocity into MetersPerSecond),
                    rotationSpeed * (maxAttainableAngularVelocity into RadiansPerSecond),
                    pose.rotation
                )
            } else {
                speeds = ChassisSpeeds(
                    forwardS * (maxAttainableLinearVelocity into MetersPerSecond),
                    strafeS * (maxAttainableLinearVelocity into MetersPerSecond),
                    rotationSpeed * (maxAttainableAngularVelocity into RadiansPerSecond)
                )
            }
            
            applyChassisSpeeds(speeds)
        }
    }
    
    fun resetOdometry(newPose: Pose2d) {
        odometry.resetPose(newPose)
        gyro.setYaw(newPose.rotation)
    }
    
    fun resetHeading(): Command {
        return Commands.runOnce({ gyro.setYaw(Rotation2d()) })
    }
    
    fun driveSysId(): Command {
        return this.run {
            modules.map { it.applyState(SwerveModuleState()) }
        }.withTimeout(3.0)
            .andThen(
                routineToApply.dynamic(SysIdRoutine.Direction.kForward),
                WaitCommand(1.0),
                routineToApply.dynamic(SysIdRoutine.Direction.kReverse),
                WaitCommand(1.0),
                routineToApply.quasistatic(SysIdRoutine.Direction.kForward),
                WaitCommand(1.0),
                routineToApply.quasistatic(SysIdRoutine.Direction.kReverse),
                PrintCommand("done!")
            )
    }
    
    fun addVisionMeasurement(pose: Pose2d, timestamp: Double, stdDevs: Vector<N3>) {
        odometry.addVisionMeasurement(pose, timestamp, stdDevs)
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
        Logger.recordOutput("$name/wheelStates", *wheelStates.toTypedArray())
        Logger.recordOutput("$name/desiredStates", *desiredStates.toTypedArray())
        Logger.recordOutput("$name/wheelPositions", *wheelPositions.toTypedArray())
        Logger.recordOutput("$name/moduleForces", *moduleForces.toTypedArray())
        Logger.recordOutput("$name/limits", SuperstructureGoals.DriveLimits.struct, limits)
    }
    
    companion object Constants {
        
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
        val moduleTranslations: List<Translation2d> = when (RobotType.type) {
            else -> listOf(
                Translation2d(Inches.of(8.864613), Inches.of(8.864613)),
                Translation2d(Inches.of(8.864613), Inches.of(-8.864613)),
                Translation2d(Inches.of(-8.864613), Inches.of(8.864613)),
                Translation2d(Inches.of(-8.864613), Inches.of(-8.864613))
            )
        }
        
        /**
         * The radius of the drivebase.
         *
         * This is the radius of the circle that the drivebase moves in. It is the distance from the center
         * of the robot to the center of a module.
         */
        val drivebaseRadius: Distance = moduleTranslations.maxOf { it.norm } measuredIn Meters
        
        /**
         * The maximum angular velocity of the drivebase.
         *
         * This is the maximum angular velocity of the drivebase in radians per second.
         */
        
        // DONT FORGET TO CHANGE BACK!
        val maxAttainableLinearVelocity = 12.4 measuredIn FeetPerSecond
        
        val maxAttainableAngularVelocity: AngularVelocity =
            (maxAttainableLinearVelocity.baseUnitMagnitude() / drivebaseRadius.baseUnitMagnitude()) measuredIn RadiansPerSecond
        
        val defaultLimits: SuperstructureGoals.DriveLimits = SuperstructureGoals.DriveLimits(
            maxLinVel = maxAttainableLinearVelocity,
            maxAngVel = maxAttainableAngularVelocity,
            maxAccel = MetersPerSecondPerSecond.of(1.0)
        )
    }
}