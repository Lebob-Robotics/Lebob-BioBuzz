package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class IntakeOverloadGuardTest {
  private boolean instantReset;

  @Before public void cooldownGuard() { // The guarded behaviour, unless a test turns the instant reset on
    instantReset = IntakeOverloadGuard.INSTANT_RESET;
    IntakeOverloadGuard.INSTANT_RESET = false;
  }

  @After public void restore() { IntakeOverloadGuard.INSTANT_RESET = instantReset; }

  @Test public void instantResetRearmsAtOnceWithNoLockout() {
    IntakeOverloadGuard.INSTANT_RESET = true;
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    for (int trip = 0; trip < 5; trip++) {
      double t = trip * 2;
      g.update(t, true, 10);
      assertTrue(g.update(t + 0.3, true, 10));
      assertEquals(0, g.cooldownRemaining(t + 0.3), 0);
      assertTrue(g.reset(t + 0.31)); // No cooldown
      assertFalse(g.isTripped());
      assertEquals(0, g.getLoad(), 0);
    }
    assertFalse(g.isLockedOut());
  }
  @Test public void freeRunningAndBriefStartupSurgesDoNotTrip() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    for (int i = 0; i <= 1000; i++) {
      assertFalse(g.update(i * 0.02, true, i < 5 ? 9 : 2));
    }
  }

  @Test public void severeJamCutsOffWithinQuarterSecond() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    assertFalse(g.update(0, true, 10.071));
    assertFalse(g.update(0.24, true, 10.071));
    assertTrue(g.update(0.25, true, 10.071));
    assertEquals("severe current", g.getReason());
  }

  @Test public void observedFourAmpLoadNoLongerTripsAtOneAndAHalfSeconds() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    // 9 Nov log: 4.1-4.45 A for 1.52 s previously tripped the sustained-current rule.
    for (int i = 0; i <= 16; i++) {
      assertFalse(g.update(i * 0.1, true, 4.45));
    }
  }

  @Test public void sustainedHeavyLoadStillTripsTheRetainedBudget() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    g.update(0, true, 6);
    assertFalse(g.update(5.9, true, 6));
    assertTrue(g.update(6.0, true, 6));
    assertEquals("accumulated load", g.getReason());
  }

  @Test public void recentBallLoadingBurstsCanRecoverWithoutTripping() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    // Shape of the 08:46 log: loading bursts around 5-6 A with 3-4 A between them.
    for (int i = 0; i <= 150; i++) {
      double t = i * 0.1;
      double amps = t < 1.5 ? 5.2 : t < 3.5 ? 3.4 : t < 4.2 ? 6.4 : 3.2;
      assertFalse("trip at " + t, g.update(t, true, amps));
    }
    assertEquals(0, g.getLoad(), 1e-9);
  }

  @Test public void repeatedPulsesCannotEraseTheLoadBudget() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    boolean trip = false;
    for (int i = 0; i < 120; i++) {
      double t = i * 0.1;
      trip |= g.update(t, i % 3 != 2, i % 3 != 2 ? 7 : 0);
    }
    assertTrue(trip);
    assertEquals("accumulated load", g.getReason());
  }

  @Test public void normalThreeToFourAmpRunningDoesNotAccumulateLoad() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    for (int i = 0; i <= 1000; i++) {
      assertFalse(g.update(i * 0.1, true, 3.5));
    }
    assertEquals(0, g.getLoad(), 0);
  }

  @Test public void currentReturningToZeroDoesNotAutomaticallyRearm() {
    IntakeOverloadGuard g = tripped();
    assertTrue(g.update(30, false, 0));
    assertTrue(g.update(31, true, 0)); // A normal or eject request cannot clear it.
    assertTrue(g.isTripped());
  }

  @Test public void resetRequiresBothCooldownAndRecoveredLoad() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    for (int i = 0; i < 100 && !g.isTripped(); i++) { g.update(i * 0.1, true, 7); }
    assertTrue(g.isTripped());
    assertFalse(g.reset(1));
    g.update(11, false, 0);
    assertFalse(g.reset(11)); // Minimum cooldown passed, retained load still too high.
    g.update(30, false, 0);
    assertTrue(g.reset(30));
    assertFalse(g.isTripped());
    assertEquals(1, g.getTrips());
  }

  @Test public void threeTripsLockOutEvenAfterCooldown() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    for (int trip = 0; trip < 3; trip++) {
      double t = trip * 40;
      g.update(t, false, 0);
      g.update(t, true, 10);
      assertTrue(g.update(t + 0.3, true, 10));
      g.update(t + 30, false, 0);
      assertEquals(trip < 2, g.reset(t + 30));
    }
    assertEquals(3, g.getTrips());
    assertTrue(g.isLockedOut());
    assertTrue(g.isTripped());
  }

  @Test public void invalidCurrentFailsClosedWhilePowered() {
    for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1}) {
      IntakeOverloadGuard g = new IntakeOverloadGuard();
      assertTrue(g.update(0, true, bad));
      assertEquals("current sensor fault", g.getReason());
    }
  }

  @Test public void turningOffCannotClearOrIncrementAnExistingTrip() {
    IntakeOverloadGuard g = tripped();
    for (int i = 1; i <= 50; i++) { assertTrue(g.update(i, false, 0)); }
    assertEquals(1, g.getTrips());
    assertEquals(0, g.getLoad(), 0);
  }

  private static IntakeOverloadGuard tripped() {
    IntakeOverloadGuard g = new IntakeOverloadGuard();
    g.update(0, true, 10);
    assertTrue(g.update(0.3, true, 10));
    return g;
  }
}
