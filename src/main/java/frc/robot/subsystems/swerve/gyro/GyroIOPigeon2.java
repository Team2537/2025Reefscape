package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Pigeon2 gyro implementation. */
public final class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yawSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<AngularVelocity> yawRateSignal;
  private final StatusSignal<AngularVelocity> pitchRateSignal;
  private final StatusSignal<AngularVelocity> rollRateSignal;

  public GyroIOPigeon2(int id) {
    pigeon = new Pigeon2(id);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    yawSignal = pigeon.getYaw().clone();
    pitchSignal = pigeon.getPitch().clone();
    rollSignal = pigeon.getRoll().clone();
    yawRateSignal = pigeon.getAngularVelocityZWorld().clone();
    pitchRateSignal = pigeon.getAngularVelocityYWorld().clone();
    rollRateSignal = pigeon.getAngularVelocityXWorld().clone();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                    yawSignal,
                    pitchSignal,
                    rollSignal,
                    yawRateSignal,
                    pitchRateSignal,
                    rollRateSignal)
                .isOK();

    inputs.yaw = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    inputs.pitch = Rotation2d.fromDegrees(pitchSignal.getValueAsDouble());
    inputs.roll = Rotation2d.fromDegrees(rollSignal.getValueAsDouble());

    inputs.yawRateRadPerSec = Math.toRadians(yawRateSignal.getValueAsDouble());
    inputs.pitchRateRadPerSec = Math.toRadians(pitchRateSignal.getValueAsDouble());
    inputs.rollRateRadPerSec = Math.toRadians(rollRateSignal.getValueAsDouble());
  }

  @Override
  public void setYaw(Rotation2d heading) {
    pigeon.setYaw(heading.getDegrees());
  }
}
