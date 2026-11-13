package org.firstinspires.ftc.teamcode.utils;

/** Stops powered standby while filling, and clears feeder pressure before a requested spin-up. */
public final class LoadingShooterInterlock {
  private static final double SAFE_LOAD_RPM = 100;
  private static final double STOPPED_SECONDS = .15;
  private static final double CLEARANCE_SECONDS = .30;
  private boolean enabled, fire, wasFire, blocked, readyToLoad, clearing;
  private double clearSince = Double.NaN;
  private double stoppedSince = Double.NaN;

  public void update(double now, boolean enabled, boolean fireAllowed, double maxWheelRpm) {
    this.enabled = enabled;
    fire = fireAllowed;
    if (!enabled) {
      blocked = clearing = wasFire = false;
      readyToLoad = true;
      clearSince = stoppedSince = Double.NaN;
      return;
    }
    if (!fireAllowed) {
      blocked = true;
      clearing = wasFire = false;
      clearSince = Double.NaN;
    } else {
      if (!wasFire) { clearSince = now; }
      wasFire = true;
      clearing = now - clearSince < CLEARANCE_SECONDS;
      blocked = clearing;
    }
    if (!fireAllowed && Double.isFinite(maxWheelRpm) && Math.abs(maxWheelRpm) <= SAFE_LOAD_RPM) {
      if (Double.isNaN(stoppedSince)) { stoppedSince = now; }
    } else {
      stoppedSince = Double.NaN;
    }
    readyToLoad = !fireAllowed && !Double.isNaN(stoppedSince)
        && now - stoppedSince >= STOPPED_SECONDS;
  }

  public boolean blocksSpin() { return blocked; }
  public boolean canLoad() { return readyToLoad; }
  public boolean clearing() { return clearing; }
  public String state() {
    return !enabled ? "DISABLED" : clearing ? "CLEARING" : fire ? "SHOOTING"
        : readyToLoad ? "LOAD_READY" : "COASTING";
  }
}
