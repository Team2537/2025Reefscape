package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.Drivebase
import java.util.Optional
import java.util.function.Supplier

class AlignmentCommand(val drivebase: Drivebase, supplier: Supplier<Optional<Pose2d>>): Command() {
    init { addRequirements(drivebase) }
    
    override fun initialize() {
        drivebase.alignmentMode = true
    }
}