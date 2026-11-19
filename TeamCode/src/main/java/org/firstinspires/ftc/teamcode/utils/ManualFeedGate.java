package org.firstinspires.ftc.teamcode.utils;

/** Waits for shooting speed once per RT hold, then permits uninterrupted feeding. */
public final class ManualFeedGate {
  private boolean feeding;

  public boolean update(boolean enabled, boolean atSpeed) {
    // No timeout: a slow first ball must never be forced into the flywheels.
    feeding = enabled && (feeding || atSpeed);
    return feeding;
  }
}
