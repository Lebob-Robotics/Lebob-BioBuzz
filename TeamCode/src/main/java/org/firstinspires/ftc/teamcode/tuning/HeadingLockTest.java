package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Heading Lock Tuning's test, without hardware: a fixed run of heading steps and moving targets (the robot as a
 * turret, as shooting on the move turns it), what the lock did in each, and in plain words what to change.
 *
 * <p>Steps measure the lock on a still target (standing shots): time to get within {@link #TOLERANCE_DEG}, overshoot
 * and where it stops. Ramps move the target at a steady rate, first one way then back (strafing past the Cell): the
 * lag behind it. In steady tracking the lock's power is KP * lag + KV * rate, so the lag at a rate gives the KV that
 * would track with none: KV + KP * lag / rate.
 */
public final class HeadingLockTest {
  /** Within this of the target counts as there: about the shot's heading tolerance at 2 m. */
  public static final double TOLERANCE_DEG = 1.5;
  static final double STEP_S = 1.6;
  static final double RAMP_LEG_S = 1.5;
  /** The start of each ramp leg, while the lock catches the new rate, is left out of its lag. */
  static final double RAMP_SETTLE_S = 0.4;

  /** One part of the run: a step to offsetDeg from the start heading, or a ramp at rateRadPerSec. */
  static final class Segment {
    final boolean ramp;
    final double value;

    Segment(boolean ramp, double value) {
      this.ramp = ramp;
      this.value = value;
    }

    double durationS() { return ramp ? 2 * RAMP_LEG_S : STEP_S; }

    String name() {
      return ramp ? String.format(Locale.US, "ramp %.1f rad/s", value)
          : String.format(Locale.US, "step to %+.0f deg", value);
    }
  }

  static final Segment[] PLAN = {
    new Segment(false, 30), new Segment(false, 0), new Segment(false, -30), new Segment(false, 0),
    new Segment(false, 90), new Segment(false, 0), new Segment(false, 10), new Segment(false, 0),
    new Segment(true, 0.5), new Segment(true, 1.0), new Segment(true, 2.0),
  };

  /** What one segment showed. Degrees and seconds; NaN where it doesn't apply. */
  public static final class Result {
    public final String name;
    public final boolean ramp;
    public final double rateRadPerSec;
    public final double reachS;
    public final double settleS;
    public final double overshootDeg;
    public final double endErrorDeg;
    public final double lagDeg; // Ramps: mean error behind the moving target
    public final double maxLagDeg;

    Result(String name, boolean ramp, double rateRadPerSec, double reachS, double settleS, double overshootDeg,
        double endErrorDeg, double lagDeg, double maxLagDeg) {
      this.name = name;
      this.ramp = ramp;
      this.rateRadPerSec = rateRadPerSec;
      this.reachS = reachS;
      this.settleS = settleS;
      this.overshootDeg = overshootDeg;
      this.endErrorDeg = endErrorDeg;
      this.lagDeg = lagDeg;
      this.maxLagDeg = maxLagDeg;
    }

    public String line() {
      if (ramp) {
        return String.format(Locale.US, "%s: lag %.1f deg (worst %.1f)", name, lagDeg, maxLagDeg);
      }
      return String.format(Locale.US, "%s: there in %s, overshoot %.1f deg, ends %.1f deg off", name,
          Double.isNaN(reachS) ? "NEVER" : String.format(Locale.US, "%.2f s", reachS), overshootDeg, endErrorDeg);
    }
  }

  private double startS = Double.NaN;
  private double startHeadingRad;
  private int index;
  private double segmentStartS;
  private double segmentStartOffsetDeg; // Where the target was as the segment began
  private StepResponse step;
  private double lagSum;
  private int lagCount;
  private double maxLag;
  private final List<Result> results = new ArrayList<>();

  /** Starts the run from the robot's heading now. */
  public void start(double nowS, double headingRad) {
    startS = nowS;
    startHeadingRad = headingRad;
    index = 0;
    results.clear();
    beginSegment(nowS, 0, 0);
  }

  public boolean running() { return !Double.isNaN(startS) && index < PLAN.length; }

  public boolean finished() { return !Double.isNaN(startS) && index >= PLAN.length; }

  public void stop() { startS = Double.NaN; }

  public List<Result> results() { return results; }

  /** "3/11 step to +30 deg", for the screen. */
  public String progress() {
    return running() ? (index + 1) + "/" + PLAN.length + " " + PLAN[index].name() : finished() ? "done" : "not run";
  }

  /** {field heading, its rate CCW rad/s} the lock should hold now; null when not running. */
  public double[] target(double nowS) {
    if (!running()) {
      return null;
    }
    double offsetDeg = targetOffsetDeg(nowS);
    return new double[] {startHeadingRad + Math.toRadians(offsetDeg), targetRate(nowS)};
  }

  private double targetOffsetDeg(double nowS) {
    Segment s = PLAN[index];
    if (!s.ramp) {
      return s.value;
    }
    double t = nowS - segmentStartS;
    double outS = Math.min(t, RAMP_LEG_S);
    double backS = Math.max(0, Math.min(t - RAMP_LEG_S, RAMP_LEG_S));
    return segmentStartOffsetDeg + Math.toDegrees(s.value * (outS - backS));
  }

  private double targetRate(double nowS) {
    Segment s = PLAN[index];
    return !s.ramp ? 0 : nowS - segmentStartS < RAMP_LEG_S ? s.value : -s.value;
  }

  /** This loop's heading (field, rad). Moves the run on when a segment's time is up. */
  public void record(double nowS, double headingRad) {
    if (!running()) {
      return;
    }
    double headingOffsetDeg = Math.toDegrees(normalize(headingRad - startHeadingRad));
    Segment s = PLAN[index];
    double t = nowS - segmentStartS;
    if (!s.ramp) {
      step.add(nowS, headingOffsetDeg);
    } else if (t % RAMP_LEG_S >= RAMP_SETTLE_S && t < s.durationS()) {
      double rate = targetRate(nowS);
      double lag = (targetOffsetDeg(nowS) - headingOffsetDeg) * Math.signum(rate);
      lagSum += lag;
      lagCount++;
      maxLag = Math.max(maxLag, Math.abs(lag));
    }
    if (t >= s.durationS()) {
      results.add(s.ramp
          ? new Result(s.name(), true, s.value, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
              lagCount == 0 ? Double.NaN : lagSum / lagCount, maxLag)
          : new Result(s.name(), false, 0, step.reachS(), step.settleS(), step.overshoot(), step.endError(),
              Double.NaN, Double.NaN));
      double endOffsetDeg = targetOffsetDeg(nowS);
      index++;
      if (running()) {
        beginSegment(nowS, endOffsetDeg, headingOffsetDeg);
      }
    }
  }

  private void beginSegment(double nowS, double targetOffsetDeg, double headingOffsetDeg) {
    segmentStartS = nowS;
    segmentStartOffsetDeg = PLAN[index].ramp ? targetOffsetDeg : PLAN[index].value;
    step = new StepResponse(nowS, headingOffsetDeg, PLAN[index].value, TOLERANCE_DEG);
    lagSum = 0;
    lagCount = 0;
    maxLag = 0;
  }

  /**
   * What to change, in plain words, from a finished run with these gains. The first line is the verdict; each
   * suggested value is also in {@code suggested} as {kp, kd, kv, ks} (unchanged where nothing is suggested).
   */
  public static List<String> advice(List<Result> results, double kp, double kd, double kv, double ks,
      double[] suggested) {
    List<String> lines = new ArrayList<>();
    suggested[0] = kp;
    suggested[1] = kd;
    suggested[2] = kv;
    suggested[3] = ks;
    double overshoot = 0;
    double slowest = 0;
    double endOff = 0;
    int steps = 0;
    boolean never = false;
    for (Result r : results) {
      if (r.ramp) {
        continue;
      }
      steps++;
      if (Math.abs(r.overshootDeg) > overshoot && !r.name.contains("+10") && !r.name.contains("-10")) {
        overshoot = Math.abs(r.overshootDeg);
      }
      never |= Double.isNaN(r.reachS);
      slowest = Math.max(slowest, Double.isNaN(r.settleS) ? HeadingLockTest.STEP_S : r.settleS);
      endOff = Math.max(endOff, Math.abs(r.endErrorDeg));
    }
    // KV from the fast ramps only: slow ones run on the least power (KS), where the lag says nothing about KV
    double kvSum = 0;
    int kvN = 0;
    double worstLag = 0;
    for (Result r : results) {
      if (!r.ramp || Double.isNaN(r.lagDeg)) {
        continue;
      }
      if (Math.abs(r.lagDeg) > Math.abs(worstLag)) {
        worstLag = r.lagDeg;
      }
      if (kv * r.rateRadPerSec > ks + 0.02) {
        kvSum += kv + kp * Math.toRadians(r.lagDeg) / r.rateRadPerSec;
        kvN++;
      }
    }
    boolean good = true;
    if (steps > 0 && overshoot > 3) {
      good = false;
      suggested[0] = round(kp * 0.8);
      suggested[1] = round(kd + 0.02);
      lines.add(String.format(Locale.US, "Overshoots %.1f deg: lower HEADING_KP to %.2f, raise HEADING_KD to %.2f",
          overshoot, suggested[0], suggested[1]));
    } else if (steps > 0 && slowest > 0.9 && overshoot < 1.5) {
      good = false;
      suggested[0] = round(kp * 1.2);
      lines.add(String.format(Locale.US, "Slow to line up (%.2f s): raise HEADING_KP to %.2f", slowest,
          suggested[0]));
    }
    if (steps > 0 && (endOff > 1.0 || never)) {
      good = false;
      suggested[3] = round(ks + 0.02);
      lines.add(String.format(Locale.US, "Stops %.1f deg short: raise HEADING_KS to %.2f", endOff, suggested[3]));
    }
    if (kvN > 0) {
      double kvNew = round(Math.max(kv * 0.5, Math.min(kv * 1.5, kvSum / kvN)));
      if (Math.abs(worstLag) > TOLERANCE_DEG && Math.abs(kvNew - kv) >= 0.01) {
        good = false;
        suggested[2] = kvNew;
        lines.add(String.format(Locale.US, "%s a moving target by %.1f deg: set HEADING_KV to %.2f",
            worstLag > 0 ? "Trails" : "Runs ahead of", Math.abs(worstLag), kvNew));
      }
    }
    lines.add(0, good ? "GOOD: nothing to change" : "CHANGE: press X to try the suggested values, A to run again");
    return lines;
  }

  private static double round(double v) {
    return Math.round(v * 100) / 100.0;
  }

  private static double normalize(double rad) {
    return Math.atan2(Math.sin(rad), Math.cos(rad));
  }
}
