package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.List;

/**
 * One step of a controlled value (a heading, a wheel speed) from where it was to a new goal: how long it took to
 * get there, how far it went past, and where it ended. Pure Java, so the tuning OpModes' verdicts are unit tested.
 */
public final class StepResponse {
  /** The end error is the mean over this last stretch of the step. */
  static final double END_WINDOW_S = 0.3;

  private final double startS;
  private final double from;
  private final double goal;
  private final double toleranceAbs;
  private final List<double[]> samples = new ArrayList<>();

  /**
   * @param toleranceAbs within this of the goal counts as there
   */
  public StepResponse(double startS, double from, double goal, double toleranceAbs) {
    this.startS = startS;
    this.from = from;
    this.goal = goal;
    this.toleranceAbs = toleranceAbs;
  }

  public void add(double nowS, double value) {
    if (!Double.isNaN(value)) {
      samples.add(new double[] {nowS - startS, value});
    }
  }

  public double goal() { return goal; }

  /** Seconds from the step until first within the tolerance; NaN if it never got there. */
  public double reachS() {
    for (double[] s : samples) {
      if (Math.abs(s[1] - goal) <= toleranceAbs) {
        return s[0];
      }
    }
    return Double.NaN;
  }

  /** Seconds from the step until it stayed within the tolerance to the end; NaN if it didn't. */
  public double settleS() {
    double settled = Double.NaN;
    for (double[] s : samples) {
      if (Math.abs(s[1] - goal) > toleranceAbs) {
        settled = Double.NaN;
      } else if (Double.isNaN(settled)) {
        settled = s[0];
      }
    }
    return settled;
  }

  /** Furthest past the goal in the step's direction, 0 if it never passed it. */
  public double overshoot() {
    double direction = Math.signum(goal - from);
    if (direction == 0) {
      direction = 1;
    }
    double most = 0;
    for (double[] s : samples) {
      most = Math.max(most, (s[1] - goal) * direction);
    }
    return most;
  }

  /** Mean of value - goal over the last END_WINDOW_S. */
  public double endError() {
    if (samples.isEmpty()) {
      return Double.NaN;
    }
    double endS = samples.get(samples.size() - 1)[0];
    double sum = 0;
    int n = 0;
    for (double[] s : samples) {
      if (s[0] >= endS - END_WINDOW_S) {
        sum += s[1] - goal;
        n++;
      }
    }
    return sum / n;
  }

  /** Largest minus smallest value over the last END_WINDOW_S: the wobble once there. */
  public double endSpread() {
    if (samples.isEmpty()) {
      return Double.NaN;
    }
    double endS = samples.get(samples.size() - 1)[0];
    double lo = Double.POSITIVE_INFINITY;
    double hi = Double.NEGATIVE_INFINITY;
    for (double[] s : samples) {
      if (s[0] >= endS - END_WINDOW_S) {
        lo = Math.min(lo, s[1]);
        hi = Math.max(hi, s[1]);
      }
    }
    return hi - lo;
  }
}
