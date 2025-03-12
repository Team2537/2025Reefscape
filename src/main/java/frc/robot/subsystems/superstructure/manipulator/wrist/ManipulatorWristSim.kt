package frc.robot.subsystems.superstructure.manipulator.wrist

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.math.units.into

class ManipulatorWristSim(
    motor: DCMotor = DCMotor.getKrakenX60Foc(1),
    gearing: Double,
    mass: Mass,
    moi: MomentOfInertia
): ManipulatorWristIO {
    private val sim: SingleJointedArmSim = SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            motor,
            moi into KilogramSquareMeters,
            gearing
        ),
        motor,
        gearing,
        Units.inchesToMeters(16.0),
        Units.degreesToRadians(0.0),
        Units.degreesToRadians(180.0),
        true,
        0.0
    )
}