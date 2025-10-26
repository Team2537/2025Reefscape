package lib.math.controllers.gains;

/** Container for feedforward constants. */
public final class FeedforwardGains {
  private final double kS;
  private final double kV;
  private final double kA;

  public FeedforwardGains() {
    this(0.0, 0.0, 0.0);
  }

  public FeedforwardGains(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public double getKS() {
    return kS;
  }

  public double getKV() {
    return kV;
  }

  public double getKA() {
    return kA;
  }
}
