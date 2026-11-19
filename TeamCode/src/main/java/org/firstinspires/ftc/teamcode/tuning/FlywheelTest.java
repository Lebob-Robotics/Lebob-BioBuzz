package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel Tuning's test, without hardware: a fixed run of speed steps across the shot table's range, then balls
 * fed one at a time, what the wheels did, and in plain words what to change in ShooterSubsystem's PIDF.
 */
public final class FlywheelTest {
  /** Within this of the target counts as there: the fire gate's band. */
  public static final double TOLERANCE_RPM = 50;
  static final double STEP_S = 2.5;
  // Big steps up spin up at full power and hand back to speed control (ShooterSubsystem's OPEN_LOOP_BELOW_RPM and
  // HANDBACK_RPM); small ones, like the table's changes while driving, are the speed control alone
  static final double[] PLAN_RPM = {2500, 3500, 4000, 3300, 3800, 3600, 3800, 3000, 3500};
  /** Steps up by more than this spin up at full power first. */
  static final double OPEN_LOOP_STEP_RPM = 400;
  /** A ball is fed this long after the wheels settled from the last. */
  public static final double BALL_SETTLE_S = 0.8;
  public static final double BALL_RPM = 3500;

  /** One speed step. */
  public static final class StepResult {
    public final double fromRpm;
    public final double toRpm;
    public final double reachS;
    public final double settleS;
    public final double overshootRpm;
    public final double endErrorRpm;
    public final double endSpreadRpm;

    public StepResult(double fromRpm, double toRpm, StepResponse r) {
      this.fromRpm = fromRpm;
      this.toRpm = toRpm;
      reachS = r.reachS();
      settleS = r.settleS();
      overshootRpm = r.overshoot();
      endErrorRpm = r.endError();
      endSpreadRpm = r.endSpread();
    }

    public String line() {
      return String.format(Locale.US, "%.0f->%.0f: there in %s, overshoot %.0f, ends %+.0f (wobble %.0f)",
          fromRpm, toRpm, Double.isNaN(reachS) ? "NEVER" : String.format(Locale.US, "%.2f s", reachS),
          overshootRpm, endErrorRpm, endSpreadRpm);
    }
  }

  /** One ball: how far the wheels dipped and how long until back within TOLERANCE_RPM. */
  public static final class BallResult {
    public final double dipRpm;
    public final double recoverS;

    public BallResult(double dipRpm, double recoverS) {
      this.dipRpm = dipRpm;
      this.recoverS = recoverS;
    }

    public String line() {
      return String.format(Locale.US, "ball: dipped %.0f rpm, back in %s", dipRpm,
          Double.isNaN(recoverS) ? "NEVER" : String.format(Locale.US, "%.2f s", recoverS));
    }
  }

  private double startS = Double.NaN;
  private int index;
  private double stepStartS;
  private StepResponse step;
  private double lastRpm;
  private final List<StepResult> steps = new ArrayList<>();

  public void start(double nowS, double rpmNow) {
    startS = nowS;
    index = 0;
    steps.clear();
    begin(nowS, rpmNow);
  }

  public boolean running() { return !Double.isNaN(startS) && index < PLAN_RPM.length; }

  public boolean finished() { return !Double.isNaN(startS) && index >= PLAN_RPM.length; }

  public void stop() { startS = Double.NaN; }

  public List<StepResult> steps() { return steps; }

  /** The target RPM now; NaN when not running. */
  public double targetRpm() { return running() ? PLAN_RPM[index] : Double.NaN; }

  public String progress() {
    return running() ? String.format(Locale.US, "%d/%d to %.0f rpm", index + 1, PLAN_RPM.length, PLAN_RPM[index])
        : finished() ? "done" : "not run";
  }

  /** This loop's measured mean RPM. Moves on when a step's time is up. */
  public void record(double nowS, double rpm) {
    lastRpm = rpm;
    if (!running()) {
      return;
    }
    step.add(nowS, rpm);
    if (nowS - stepStartS >= STEP_S) {
      steps.add(new StepResult(index == 0 ? Double.NaN : PLAN_RPM[index - 1], PLAN_RPM[index], step));
      index++;
      if (running()) {
        begin(nowS, rpm);
      }
    }
  }

  private void begin(double nowS, double rpmNow) {
    stepStartS = nowS;
    step = new StepResponse(nowS, rpmNow, PLAN_RPM[index], TOLERANCE_RPM);
  }

  /**
   * What to change, from the steps (the first is from wherever the wheels were, so it is left out) and balls, with
   * these gains. First line is the verdict. {@code suggested} gets {p, f, handback}.
   */
  public static List<String> advice(List<StepResult> steps, List<BallResult> balls, double p, double f,
      double handbackRpm, double[] suggested) {
    List<String> lines = new ArrayList<>();
    suggested[0] = p;
    suggested[1] = f;
    suggested[2] = handbackRpm;
    double overshoot = 0;
    double spinUpOvershoot = 0;
    double slowPerKrpm = 0;
    double spread = 0;
    double fRatioSum = 0;
    int n = 0;
    for (int i = 1; i < steps.size(); i++) {
      StepResult s = steps.get(i);
      if (s.toRpm - s.fromRpm > OPEN_LOOP_STEP_RPM) {
        spinUpOvershoot = Math.max(spinUpOvershoot, s.overshootRpm);
      } else {
        overshoot = Math.max(overshoot, s.overshootRpm);
      }
      double size = Math.abs(s.toRpm - s.fromRpm) / 1000.0;
      double reach = Double.isNaN(s.reachS) ? STEP_S : s.reachS;
      slowPerKrpm = Math.max(slowPerKrpm, reach / Math.max(0.3, size));
      spread = Math.max(spread, s.endSpreadRpm);
      if (!Double.isNaN(s.endErrorRpm)) {
        fRatioSum += s.toRpm / (s.toRpm + s.endErrorRpm);
        n++;
      }
    }
    boolean good = true;
    if (spinUpOvershoot > 150) {
      good = false;
      suggested[2] = Math.min(400, handbackRpm + 50);
      lines.add(String.format(Locale.US, "Full-power spin-up overshoots %.0f rpm: raise HANDBACK_RPM to %.0f",
          spinUpOvershoot, suggested[2]));
    }
    if (overshoot > 120 || spread > 150) {
      good = false;
      suggested[0] = Math.round(p * 0.8);
      lines.add(String.format(Locale.US, "Overshoots %.0f rpm / wobbles %.0f: lower VELOCITY_P to %.0f", overshoot,
          spread, suggested[0]));
    } else if (slowPerKrpm > 1.2) {
      good = false;
      suggested[0] = Math.round(Math.min(240, p * 1.2));
      lines.add(String.format(Locale.US, "Slow to change speed (%.1f s per 1000 rpm): raise VELOCITY_P to %.0f",
          slowPerKrpm, suggested[0]));
    }
    if (n > 0) {
      double ratio = fRatioSum / n;
      if (Math.abs(ratio - 1) > 0.008) {
        good = false;
        suggested[1] = Math.round(f * ratio * 100) / 100.0;
        lines.add(String.format(Locale.US, "Holds %s the target: set VELOCITY_F to %.2f",
            ratio < 1 ? "above" : "below", suggested[1]));
      }
    }
    double slowest = 0;
    double deepest = 0;
    for (BallResult b : balls) {
      slowest = Math.max(slowest, Double.isNaN(b.recoverS) ? 2 : b.recoverS);
      deepest = Math.max(deepest, b.dipRpm);
    }
    if (!balls.isEmpty()) {
      lines.add(String.format(Locale.US, "Balls: deepest dip %.0f rpm, slowest recovery %.2f s%s", deepest, slowest,
          slowest > 0.45 ? " (slow: keep VOLLEY_FAST_RECOVERY on, raise VELOCITY_P)" : ""));
    }
    lines.add(0, good ? "GOOD: nothing to change" : "CHANGE: press X to try the suggested values, A to run again");
    return lines;
  }
}
