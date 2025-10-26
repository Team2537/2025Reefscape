package lib.math.controllers.gains;

/**
 * Pairing of PID and feedforward gains for convenience.
 */
public final class ControllerGains {
  private final PIDGains pid;
  private final FeedforwardGains feedforward;

  public ControllerGains(PIDGains pid, FeedforwardGains feedforward) {
    this.pid = pid;
    this.feedforward = feedforward;
  }

  public PIDGains getPid() {
    return pid;
  }

  public FeedforwardGains getFeedforward() {
    return feedforward;
  }
}
