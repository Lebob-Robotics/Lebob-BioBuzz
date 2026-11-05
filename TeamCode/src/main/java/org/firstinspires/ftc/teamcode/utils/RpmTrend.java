package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayDeque;

/**
 * Where the flywheels' speed is heading: the slope over the last {@code windowS}, carried forward. A wheel
 * recovering from the last ball passes through the target still speeding up and overshoots it by 100-150 RPM,
 * which the speed now does not show (5 Nov, 080706: fed at +17 RPM, left at +130).
 */
public final class RpmTrend {
  private final double windowS;
  private final ArrayDeque<double[]> samples = new ArrayDeque<>(); // {time s, rpm}, oldest first

  public RpmTrend(double windowS) {
    this.windowS = windowS;
  }

  public void update(double nowS, double rpm) {
    samples.addLast(new double[] {nowS, rpm});
    while (nowS - samples.peekFirst()[0] > windowS) {
      samples.removeFirst();
    }
  }

  /** The speed lookaheadS from the last sample, at the window's slope; the last speed if there is no slope yet. */
  public double predicted(double lookaheadS) {
    if (samples.isEmpty()) {
      return 0.0;
    }
    double[] first = samples.peekFirst();
    double[] last = samples.peekLast();
    double spanS = last[0] - first[0];
    if (spanS <= 0.0) {
      return last[1];
    }
    return last[1] + (last[1] - first[1]) / spanS * lookaheadS;
  }
}
