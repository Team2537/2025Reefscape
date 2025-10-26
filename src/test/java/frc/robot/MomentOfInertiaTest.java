package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.junit.jupiter.api.Test;

class MomentOfInertiaTest {
  @Test
  void armMomentOfInertiaIsNonNegative() {
    MomentOfInertia moi = Constants.ManipulatorConstants.SIM_ROLLER_MOI;
    assertTrue(moi.in(Units.KilogramSquareMeters) >= 0.0);
  }
}
