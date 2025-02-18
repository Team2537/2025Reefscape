package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CoralSimulator
import frc.robot.MechanismVisualizer
import frc.robot.Robot.drivebase
import frc.robot.subsystems.superstructure.arm.Arm
import frc.robot.subsystems.superstructure.elevator.Elevator
import frc.robot.subsystems.superstructure.gripper.Gripper
import lib.commands.not
import lib.math.units.degrees
import lib.math.units.meters
import lib.math.units.radians
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Superstructure {
    val elevator: Elevator = Elevator()
    val arm: Arm = Arm()
    val gripper: Gripper = Gripper()
    
    private var lastRequest: SuperstructureGoal.SuperstructureState = SuperstructureGoal.STOW
    
    val gamepieceSimulator = CoralSimulator(
        { MechanismVisualizer.mechanismPoses[2] },
        drivebase::pose,
        Inches.of(24.0)
    )
    
    
    private val readyToScore: Trigger = Trigger { SmartDashboard.getBoolean("shouldScore", false) }
    
    
    fun periodic() {
        Logger.recordOutput("superstructure/setpoint", SuperstructureGoal.SuperstructureState.struct, lastRequest)
        gamepieceSimulator.update()
    }
}