package lib.math.controllers.gains;

/** Simple container for PID constants. */
public final class PIDGains {
  private final double kP;
  private final double kI;
  private final double kD;

  public PIDGains() {
    this(0.0, 0.0, 0.0);
  }

  public PIDGains(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public double getKP() {
    return kP;
  }

  public double getKI() {
    return kI;
  }

  public double getKD() {
    return kD;
  }
}
