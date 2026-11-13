package org.firstinspires.ftc.teamcode.subsystems;

/** Current-only intake protection: its encoder is not reliable enough for stall detection. */
public final class IntakeOverloadGuard {
  // 9 Nov loading logs: tolerate 3-4 A running and brief 5-7 A ball-handling bursts.
  // These are overload limits, not a measurement of winding temperature.
  public static final double SEVERE_AMPS = 8.0;
  public static final double SEVERE_SECONDS = 0.25;
  public static final double LOAD_BASELINE_AMPS = 4.0;
  public static final double LOAD_BUDGET = 120.0; // Integral of excess A^2 above the baseline, in A^2 s
  public static final double COOLDOWN_SECONDS = 10.0;
  public static final int MAX_TRIPS = 3;
  // LB re-arms a trip at once and clears the load, with no cooldown and no lockout: the drivers' call, at their own
  // risk (13 Nov). Today's TeleOp ran the intake at a median 5-5.7 A (balls held against it), which spent the load
  // budget in ~8 s, and three trips locked it out for the rest of the run (175517 at 136 s). False: the cooldown,
  // recovered load and MAX_TRIPS lockout as before
  public static boolean INSTANT_RESET = true;
  private static final double RECOVERY_PER_SECOND = 10.0;
  private static final double RESET_BUDGET = 5.0;

  private double lastTime = Double.NaN;
  private double severeSince = Double.NaN;
  private double load;
  private double trippedAt = Double.NaN;
  private int trips;
  private boolean tripped;
  private String reason = "ok";

  /** Called every loop, including while off: stops/reversals never erase the retained load budget. */
  public boolean update(double now, boolean powered, double amps) {
    double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, now - lastTime);
    lastTime = now;
    boolean valid = Double.isFinite(amps) && amps >= 0;
    double excess = powered && valid ? amps * amps - LOAD_BASELINE_AMPS * LOAD_BASELINE_AMPS : -RECOVERY_PER_SECOND;
    load = Math.max(0, Math.min(LOAD_BUDGET, load + (excess > 0 ? excess : -RECOVERY_PER_SECOND) * dt));
    if (tripped) {
      return true;
    }
    if (powered && !valid) {
      trip(now, "current sensor fault");
    } else {
      severeSince = since(powered && amps >= SEVERE_AMPS, severeSince, now);
      if (now - severeSince >= SEVERE_SECONDS) {
        trip(now, "severe current");
      } else if (load >= LOAD_BUDGET) {
        trip(now, "accumulated load");
      }
    }
    return tripped;
  }

  private static double since(boolean high, double started, double now) {
    return high ? Double.isNaN(started) ? now : started : Double.NaN;
  }

  private void trip(double now, String why) {
    tripped = true;
    trips++;
    trippedAt = now;
    reason = why;
  }

  /** Explicit driver action only. Failed/held reset attempts cannot clear the trip or load history. */
  public boolean reset(double now) {
    if (INSTANT_RESET && tripped) {
      tripped = false;
      load = 0;
      severeSince = Double.NaN;
      reason = "ok";
      return true;
    }
    if (!tripped || isLockedOut() || now - trippedAt < COOLDOWN_SECONDS || load > RESET_BUDGET) {
      return false;
    }
    tripped = false;
    severeSince = Double.NaN;
    reason = "ok";
    return true;
  }

  public boolean isTripped() { return tripped; }
  public boolean isLockedOut() { return !INSTANT_RESET && trips >= MAX_TRIPS; }
  public int getTrips() { return trips; }
  public double getLoad() { return load; }
  public String getReason() { return reason; }
  public double cooldownRemaining(double now) {
    if (INSTANT_RESET) {
      return 0;
    }
    return tripped ? Math.max(Math.max(0, COOLDOWN_SECONDS - (now - trippedAt)),
        Math.max(0, load - RESET_BUDGET) / RECOVERY_PER_SECOND) : 0;
  }
}
