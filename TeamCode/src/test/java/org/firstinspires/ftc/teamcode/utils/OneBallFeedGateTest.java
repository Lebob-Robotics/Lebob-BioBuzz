package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

public class OneBallFeedGateTest {
  private static final double TARGET = 3500;

  @Test public void waitsForSpeedHeldBeforeFeeding() {
    OneBallFeedGate gate = new OneBallFeedGate();
    assertFalse(gate.update(0.00, true, true, 3505, TARGET));
    assertFalse(gate.update(0.05, true, true, 3505, TARGET));
    assertTrue(gate.update(0.10, true, true, 3505, TARGET));
    assertTrue(gate.feedStarted());
  }

  @Test public void aimedGateWithNoWaitFeedsOnTheFirstReadyLoop() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0);
    assertFalse(gate.update(0.00, true, false, 3300, TARGET));
    assertTrue(gate.update(0.08, true, true, 3505, TARGET));
    assertTrue(gate.feedStarted());
  }

  @Test public void keepsFeedingThroughSpeedReadingSteps() {
    OneBallFeedGate gate = new OneBallFeedGate();
    gate.update(0.00, true, true, 3505, TARGET);
    gate.update(0.10, true, true, 3505, TARGET);
    // The reading steps to 3443 (outside a 50 band): the feed must not stop and restart
    assertTrue(gate.update(0.11, true, false, 3443, TARGET));
    assertTrue(gate.update(0.12, true, true, 3566, TARGET));
    assertTrue(gate.update(0.13, true, false, 3443, TARGET));
  }

  @Test public void pausesAfterEachBallUntilBackToSpeed() {
    OneBallFeedGate gate = new OneBallFeedGate();
    gate.update(0.00, true, true, 3505, TARGET);
    assertTrue(gate.update(0.10, true, true, 3505, TARGET));
    assertFalse(gate.update(0.15, true, false, 3075, TARGET)); // The ball took the wheels down
    assertTrue(gate.ballLeft());
    assertFalse(gate.update(0.30, true, false, 3382, TARGET));
    assertFalse(gate.update(0.40, true, true, 3505, TARGET)); // Back, but not yet held
    assertTrue(gate.update(0.51, true, true, 3505, TARGET)); // Next ball at full speed
    assertTrue(gate.feedStarted());
  }

  @Test public void oneBallModeStopsAfterOneBallUntilReleased() {
    OneBallFeedGate gate = new OneBallFeedGate();
    gate.update(0.00, true, true, 3505, TARGET, true);
    assertTrue(gate.update(0.10, true, true, 3505, TARGET, true));
    assertFalse(gate.update(0.15, true, false, 3075, TARGET, true)); // The ball left
    assertTrue(gate.ballLeft());
    assertFalse(gate.update(0.40, true, true, 3505, TARGET, true));
    assertFalse(gate.update(1.00, true, true, 3505, TARGET, true)); // Still held: no second ball
    assertFalse(gate.update(1.05, false, true, 3505, TARGET, true)); // Released
    gate.update(1.10, true, true, 3505, TARGET, true);
    assertTrue(gate.update(1.20, true, true, 3505, TARGET, true)); // Pressed again: the next ball
  }

  @Test public void pullingTheTriggerAfterAOneBallShotFeedsTheRest() {
    OneBallFeedGate gate = new OneBallFeedGate();
    gate.update(0.00, true, true, 3505, TARGET, true);
    assertTrue(gate.update(0.10, true, true, 3505, TARGET, true));
    assertFalse(gate.update(0.15, true, false, 3075, TARGET, true)); // The ball left
    assertFalse(gate.update(0.40, true, true, 3505, TARGET, true)); // Bumper still held: done
    // Trigger pulled with the bumper still held: no longer one ball, still held
    assertFalse(gate.update(0.45, true, true, 3505, TARGET, false)); // Not yet held at speed
    assertTrue(gate.update(0.55, true, true, 3505, TARGET, false));
    assertTrue(gate.feedStarted());
  }

  @Test public void aFeedWithNoBallStopsOnceNoLongerReady() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, 0.4);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertTrue(gate.update(0.20, true, false, 3443, TARGET)); // Too soon: a ball may be on its way
    assertTrue(gate.update(0.45, true, true, 3505, TARGET)); // Still ready: keeps feeding
    assertFalse(gate.update(0.50, true, false, 3505, TARGET)); // No ball, aim gone: stop
    assertFalse(gate.ballLeft());
    assertTrue(gate.update(0.60, true, true, 3505, TARGET)); // Ready again
    assertTrue(gate.feedStarted());
  }

  @Test public void aFeedStopsAtOnceWhenTheShotIsLost() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, 0.4);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET, false, false));
    assertTrue(gate.update(0.05, true, false, 3443, TARGET, false, false)); // Speed step alone: keeps going
    assertFalse(gate.update(0.10, true, false, 3505, TARGET, false, true)); // Hood or aim gone: stop
    assertFalse(gate.ballLeft());
    assertTrue(gate.update(0.20, true, true, 3505, TARGET, false, false));
  }

  @Test public void aBallFedAboveTargetIsSeenByItsDropFromThePeak() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0);
    assertTrue(gate.update(0.00, true, true, 3843, 3731));
    assertTrue(gate.update(0.10, true, true, 3843, 3731));
    assertFalse(gate.update(0.20, true, true, 3660, 3731)); // 183 off the peak, still within 200 of target
    assertTrue(gate.ballLeft());
  }

  @Test public void aFeedStopsAfterItsPulseAndCatchesTheBallLeavingDuringThePause() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, 0.4, 0.30, 0.25, 0.35);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertTrue(gate.update(0.20, true, true, 3505, TARGET));
    assertFalse(gate.update(0.30, true, true, 3505, TARGET)); // Pulse over: the next ball is not pushed in
    assertFalse(gate.ballLeft());
    assertFalse(gate.update(0.40, true, true, 3200, TARGET)); // The first ball's dip, late
    assertTrue(gate.ballLeft());
  }

  @Test public void noBallOutAfterThePauseFeedsAgainForLonger() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, 0.4, 0.30, 0.25, 0.35);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertFalse(gate.update(0.30, true, true, 3505, TARGET));
    assertFalse(gate.update(0.55, true, true, 3505, TARGET)); // Pause over, nothing left
    assertTrue(gate.update(0.60, true, true, 3505, TARGET)); // Again
    assertTrue(gate.feedStarted());
    assertTrue(gate.update(0.95, true, true, 3505, TARGET)); // 0.35 s in: this pulse is 0.40
    assertFalse(gate.update(1.00, true, true, 3505, TARGET));
  }

  @Test public void theNextFeedWaitsTheGapAfterABall() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, 0.4, 0.30, 0.25, 0.35);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertFalse(gate.update(0.20, true, false, 3075, TARGET));
    assertTrue(gate.ballLeft());
    assertFalse(gate.update(0.28, true, true, 3505, TARGET)); // Ready again at once: too soon
    assertFalse(gate.update(0.50, true, true, 3505, TARGET));
    assertTrue(gate.update(0.56, true, true, 3505, TARGET));
  }

  @Test public void releasingStopsAtOnce() {
    OneBallFeedGate gate = new OneBallFeedGate();
    gate.update(0.00, true, true, 3505, TARGET);
    assertTrue(gate.update(0.10, true, true, 3505, TARGET));
    assertFalse(gate.update(0.11, false, true, 3505, TARGET));
  }
}
