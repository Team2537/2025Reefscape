package frc.robot.commands.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.Drivebase
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class WheelRadiusCharacterization(
    private val drivebase: Drivebase,
    private val direction: Direction
): Command() {
    val characterizationSpeed = RadiansPerSecond.of(1.0)

    val omegaLimiter = SlewRateLimiter(1.0)
    var startWheelPositions: List<Angle> = listOf()

    init {
        addRequirements(drivebase)
    }

    var lastGyroYawRads = 0.0
    var gyroYawAccumRads = 0.0

    override fun initialize() {
        startWheelPositions = drivebase.wheelRadiusCharacterizationAngles
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
        var averageWheelPositionRad = 0.0

        startWheelPositions.zip(wheelPositions).forEach {
            averageWheelPositionRad += abs(wheelPositions[0].into(Radians) - wheelPositions[1].into(Radians))
        }

        averageWheelPositionRad /= 4.0

        val currentEffectiveWheelRadiusMeters = (gyroYawAccumRads * (drivebase.drivebaseRadius into Meters)) / averageWheelPositionRad

        Logger.recordOutput("${drivebase.name}/radiusCharacterization/wheelRadius", currentEffectiveWheelRadiusMeters)


    }

    enum class Direction(val i: Int) {
        CLOCKWISE(-1),
        COUNTERCLOCKWISE(1)
    }

}