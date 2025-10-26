package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;

/** Pigeon2 gyro implementation. */
public final class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> pitchSignal;
  private final StatusSignal<Double> rollSignal;
  private final StatusSignal<Double> yawRateSignal;
  private final StatusSignal<Double> pitchRateSignal;
  private final StatusSignal<Double> rollRateSignal;

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

    inputs.yaw = Rotation2d.fromDegrees(yawSignal.getValue());
    inputs.pitch = Rotation2d.fromDegrees(pitchSignal.getValue());
    inputs.roll = Rotation2d.fromDegrees(rollSignal.getValue());

    inputs.yawRateRadPerSec = Math.toRadians(yawRateSignal.getValue());
    inputs.pitchRateRadPerSec = Math.toRadians(pitchRateSignal.getValue());
    inputs.rollRateRadPerSec = Math.toRadians(rollRateSignal.getValue());
  }

  @Override
  public void setYaw(Rotation2d heading) {
    pigeon.setYaw(heading.getDegrees());
  }
}
