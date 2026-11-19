package org.firstinspires.ftc.teamcode.utils;

/** Nonblocking coast-before-reverse interlock, followed by a gradual restart. */
public final class MotorReversalGuard {
  private final double coastSeconds;
  private final double rampSeconds;
  private final double stoppedSpeed;
  private int direction;
  private double coastStart = Double.NaN;
  private double rampStart = Double.NaN;

  public MotorReversalGuard(double coastSeconds, double rampSeconds, double stoppedSpeed) {
    this.coastSeconds = coastSeconds;
    this.rampSeconds = rampSeconds;
    this.stoppedSpeed = stoppedSpeed;
  }

  /** Returns a 0..1 command multiplier. Speed must be the largest absolute motor speed. */
  public double update(int requestedDirection, double now, double speed) {
    if (requestedDirection == 0) {
      coastStart = Double.NaN;
      return 0;
    }
    // Keep startup direction unknown until measured inertia has cleared, even if requests change.
    if (direction == 0) {
      if (Double.isNaN(coastStart) && Double.isFinite(speed) && speed <= stoppedSpeed) {
        direction = requestedDirection;
        return 1;
      }
      if (Double.isNaN(coastStart)) {
        coastStart = now;
      }
      if (now - coastStart < coastSeconds || !Double.isFinite(speed) || speed > stoppedSpeed) {
        return 0;
      }
      direction = requestedDirection;
      coastStart = Double.NaN;
      rampStart = now;
    }
    if (direction != 0 && requestedDirection != direction) {
      if (Double.isNaN(coastStart)) {
        coastStart = now;
      }
      if (now - coastStart < coastSeconds || !Double.isFinite(speed) || speed > stoppedSpeed) {
        return 0;
      }
      direction = requestedDirection;
      coastStart = Double.NaN;
      rampStart = now;
    } else {
      direction = requestedDirection;
      coastStart = Double.NaN;
    }
    if (Double.isNaN(rampStart)) {
      return 1;
    }
    return Math.max(0, Math.min(1, (now - rampStart) / rampSeconds));
  }
}
