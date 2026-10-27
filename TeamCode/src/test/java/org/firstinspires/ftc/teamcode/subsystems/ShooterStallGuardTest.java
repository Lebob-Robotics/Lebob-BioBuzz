package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/** Currents and speeds are from the Mark 3 PsiKit logs of 25 Oct. */
public class ShooterStallGuardTest {
  private static final double STEP_S = 0.1;

  private ShooterStallGuard guard;
  private double now;

  @Before
  public void setUp() {
    guard = new ShooterStallGuard();
    now = 0;
  }

  /** Feeds the same reading to both motors for a duration, returning the last result. */
  private boolean run(double seconds, double amps, double velocity) {
    return run(seconds, true, amps, velocity, amps, velocity);
  }

  private boolean run(double seconds, boolean powered,
      double leftAmps, double leftVel, double rightAmps, double rightVel) {
    boolean off = guard.isTripped();
    for (double end = now + seconds; now < end - 1e-9;) {
      now += STEP_S;
      off = guard.update(now, powered, leftAmps, leftVel, rightAmps, rightVel);
    }
    return off;
  }

  @Test
  public void runsWhileSpinningFreely() {
    assertFalse(run(30.0, 0.4, 2360));
    assertEquals(0, guard.getTrips());
  }

  @Test
  public void tripsOnSlowOverloadFromBackwardsPulleys() {
    // Both motors at 6.8 A turning 640 ticks/s: still moving, but cooking.
    assertFalse(run(1.0, 6.8, 640));
    assertTrue(run(1.0, 6.8, 640));
    assertEquals(1, guard.getTrips());
  }

  @Test
  public void tripsOnOneMotorStalledWhileTheOtherIsDead() {
    // Left motor dead (0.1 A), right stalled at 9.2 A and 0 ticks/s.
    assertTrue(run(2.0, true, 0.1, 0, 9.2, 0));
  }

  @Test
  public void ignoresSpinUpShorterThanStallTime() {
    assertFalse(run(ShooterStallGuard.STALL_TIME_S - 0.3, 9.0, 300));
    assertFalse(run(10.0, 1.5, 2300));
  }

  @Test
  public void ignoresReverseWhenFast() {
    assertFalse(run(5.0, 2.0, -2300));
  }

  @Test
  public void stoppingClearsTheStallTimer() {
    assertFalse(run(1.0, 6.8, 640));
    assertFalse(run(0.5, false, 0, 0, 0, 0));
    assertFalse(run(1.0, 6.8, 640));
  }

  @Test
  public void staysTrippedUntilRearmed() {
    assertTrue(run(2.0, 6.8, 640));
    assertTrue(run(5.0, 0, 0));
    guard.rearm();
    assertFalse(run(1.0, 0.4, 2360));
  }

  @Test
  public void locksOutAfterMaxTrips() {
    for (int i = 0; i < ShooterStallGuard.MAX_TRIPS; i++) {
      guard.rearm();
      assertTrue(run(2.0, 6.8, 640));
    }
    assertTrue(guard.isLockedOut());
    guard.rearm();
    assertTrue(guard.isTripped());
    assertTrue(run(5.0, 0.4, 2360));
  }
}
