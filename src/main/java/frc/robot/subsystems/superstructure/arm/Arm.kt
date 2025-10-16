package frc.robot.subsystems.superstructure.arm

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants.ArmConstants
import frc.robot.RobotType
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Arm : SubsystemBase() {
    val io: ArmIO = when (RobotType.mode) {
        RobotType.Mode.REAL -> ArmIOReal()
        RobotType.Mode.SIMULATION -> ArmIOSim()
        RobotType.Mode.REPLAY -> object : ArmIO {}
    }

    val inputs = ArmIO.ArmInputs()

    private val sysIdRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null,
            { state -> Logger.recordOutput("arm/sysid", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            { voltage: Voltage -> io.setVoltage(voltage) },
            null,
            this
        )
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Arm", inputs)

        val angleRad = inputs.angle.baseUnitMagnitude()
        val velocityRadPerSec = inputs.angularVelocity.baseUnitMagnitude()
        val applied = inputs.appliedVoltage.baseUnitMagnitude()

        Logger.recordOutput("Arm/AngleRad", angleRad)
        Logger.recordOutput("Arm/VelocityRadPerSec", velocityRadPerSec)
        Logger.recordOutput("Arm/AppliedVolts", applied)

    }

    private fun clampAngle(angle: Angle): Angle {
        val min = ArmConstants.MIN_ANGLE.baseUnitMagnitude()
        val max = ArmConstants.MAX_ANGLE.baseUnitMagnitude()
        val rad = angle.baseUnitMagnitude()
        val clamped = rad.coerceIn(min, max)
        return Radians.of(clamped)
    }

    fun setBrakeMode(brake: Boolean) { io.setBrakeMode(brake) }

    fun setVoltage(voltage: Voltage) { io.setVoltage(voltage) }

    fun stop() { io.stop() }

    fun setTargetAngle(angle: Angle) { io.setTargetAngle(clampAngle(angle)) }

    fun getMoveToAngleCommand(angleSupplier: Supplier<Angle>): Command {
        return runOnce { setTargetAngle(angleSupplier.get()) }
    }

    fun getDynamicSysID(direction: Direction): Command = sysIdRoutine.dynamic(direction)
    fun getQuasistaticSysID(direction: Direction): Command = sysIdRoutine.quasistatic(direction)
}

