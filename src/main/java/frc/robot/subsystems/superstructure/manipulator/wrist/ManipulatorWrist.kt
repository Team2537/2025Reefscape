package frc.robot.subsystems.superstructure.manipulator.wrist

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.MechanismVisualizer
import frc.robot.RobotType
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class ManipulatorWrist: SubsystemBase("wrist") {
    val io: ManipulatorWristIO = when(RobotType.mode) {
        RobotType.Mode.SIMULATION -> ManipulatorWristSim(
            gearing = 40.0,
            mass = Pounds.of(6.0),
            moi = KilogramSquareMeters.of(0.055093356),
            pidGains = PIDGains(100.0, 0.0, 0.0)
        )
        RobotType.Mode.REAL -> ManipulatorWristKraken(
            motorID = 22,
            pidGains = PIDGains(kP = 40.0),
            ffGains = FeedforwardGains(kS = 0.12782, kV = 4.97),
            inverted = true,
            gearing = 40.0,
            cruiseVelocity = RotationsPerSecond.of(2.0),
            acceleration = RotationsPerSecondPerSecond.of(4.0)
        )
        else -> object : ManipulatorWristIO {}
    }

    val inputs: ManipulatorWristIO.ManipulatorWristInputs = ManipulatorWristIO.ManipulatorWristInputs()


    fun getSendToAngleCmd(angleSupplier: Supplier<Angle>): Command {
        return runOnce { io.setTargetPosition(angleSupplier.get()) }
    }

    fun getManualControlCmd(voltageSupplier: Supplier<Double>): Command {
        return run { io.setVoltage(voltageSupplier.get() measuredIn Volts) }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("manipulator/$name", inputs)

        MechanismVisualizer.setArmAngle(inputs.angle)
    }
}