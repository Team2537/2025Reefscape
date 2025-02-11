package frc.robot.commands.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class WheelRadiusCharacterization(
    private val drivebase: Drivebase,
    private val direction: Direction,
    private val characterizationSpeed: AngularVelocity = RadiansPerSecond.of(1.0)
): Command() {
    private val omegaLimiter = SlewRateLimiter(1.0)
    private var startWheelPositions: List<Angle> = listOf()

    init {
        addRequirements(drivebase)
    }

    private var lastGyroYawRads = 0.0
    private var gyroYawAccumRads = 0.0

    override fun initialize() {
        startWheelPositions = drivebase.wheelRadiusCharacterizationAngles.map { it.copy() }
        gyroYawAccumRads = 0.0

        lastGyroYawRads = drivebase.gyroInputs.yaw.radians

        omegaLimiter.reset(0.0)
    }

    override fun execute() {
        drivebase.runWheelRadiusCharacterization(characterizationSpeed.times(direction.i.toDouble()))

        gyroYawAccumRads += MathUtil.angleModulus(
            drivebase.gyroInputs.yaw.radians - lastGyroYawRads
        )

        lastGyroYawRads = drivebase.gyroInputs.yaw.radians

        val wheelPositions = drivebase.wheelRadiusCharacterizationAngles
        val averageWheelPositionRad  = startWheelPositions.zip(wheelPositions).sumOf { (start, current) ->
            abs(current.into(Radians) - start.into(Radians))
        } / wheelPositions.size

        Logger.recordOutput("${drivebase.name}/radiusCharacterization/averageWheelPositionRad", averageWheelPositionRad)
        

        val currentEffectiveWheelRadiusMeters = ((gyroYawAccumRads * (drivebase.drivebaseRadius into Meters)) / averageWheelPositionRad) * direction.i

        Logger.recordOutput("${drivebase.name}/radiusCharacterization/wheelRadius", currentEffectiveWheelRadiusMeters)


    }

    enum class Direction(val i: Int) {
        CLOCKWISE(-1),
        COUNTERCLOCKWISE(1)
    }

    override fun end(interrupted: Boolean) {
        drivebase.applyChassisSpeeds(ChassisSpeeds())
    }

}