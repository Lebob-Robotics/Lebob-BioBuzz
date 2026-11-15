package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
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

  @Test public void feedSpacingHoldsEvenWhenNoBallIsSeenLeaving() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, Double.POSITIVE_INFINITY, 0.2, 0.1, 0);
    gate.setMinFeedSpacingS(1.6);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertFalse(gate.update(0.25, true, true, 3505, TARGET)); // Pulse ran out with no dip: paused
    assertFalse(gate.update(0.40, true, true, 3505, TARGET)); // Pause over, but too soon after the last feed
    assertFalse(gate.update(1.50, true, true, 3505, TARGET));
    assertTrue(gate.update(1.61, true, true, 3505, TARGET));
    assertTrue(gate.feedStarted());
  }

  @Test public void feederEmptyAfterAPulseWithNoBallUntilOneLeaves() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, Double.POSITIVE_INFINITY, 0.3, 0.35, 0);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertFalse(gate.update(0.31, true, true, 3505, TARGET)); // Paused: a ball may still be on its way
    assertFalse(gate.feederEmpty());
    assertFalse(gate.update(0.67, true, false, 3505, TARGET)); // Pause over with no dip
    assertTrue(gate.feederEmpty());
    assertTrue(gate.update(0.70, true, true, 3505, TARGET)); // Longer pulse
    assertFalse(gate.update(0.90, true, false, 3075, TARGET));
    assertTrue(gate.ballLeft());
    assertFalse(gate.feederEmpty());
  }

  @Test public void shorterPulsePausesSooner() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, Double.POSITIVE_INFINITY, 0.3, 0.35, 0);
    gate.setPulseS(0.15);
    assertTrue(gate.update(0.00, true, true, 3505, TARGET));
    assertTrue(gate.update(0.10, true, true, 3505, TARGET));
    assertFalse(gate.update(0.16, true, true, 3505, TARGET)); // Paused, waiting for the dip
    assertFalse(gate.update(0.25, true, false, 3075, TARGET));
    assertTrue(gate.ballLeft());
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

  @Test public void countsFeedsAndBallsInOnePressAndResetsOnRelease() {
    OneBallFeedGate gate = new OneBallFeedGate(0.0, Double.POSITIVE_INFINITY, 0.3, 0.2, 0.05);
    assertTrue(gate.update(0.00, true, true, 3510, TARGET));
    assertEquals(1, gate.feedStarts());
    assertEquals(0, gate.ballsLeft());
    gate.update(0.02, true, true, 3560, TARGET); // Peak during the feed
    gate.update(0.04, true, false, 3380, TARGET); // 180 under the peak: the ball left
    assertTrue(gate.ballLeft());
    assertEquals(1, gate.ballsLeft());
    assertEquals(3560, gate.peakAtLeaveRpm(), 1e-9);
    gate.update(0.06, true, false, 3200, TARGET); // Still falling
    gate.update(0.08, true, false, 3300, TARGET); // Recovering: the dip keeps its deepest
    assertEquals(360, gate.lastDipDepthRpm(), 1e-9);
    assertTrue(gate.update(0.15, true, true, 3500, TARGET)); // Back at speed and past the gap: the next ball
    assertEquals(2, gate.feedStarts());
    gate.update(0.20, false, true, 3500, TARGET);
    assertEquals(0, gate.feedStarts());
    assertEquals(0, gate.ballsLeft());
    assertTrue(Double.isNaN(gate.lastDipDepthRpm()));
  }
  @Test public void volleyRejectsTheTwoShallowFalseCountsFrom203128() {
    OneBallFeedGate gate = volleyGate();
    double[][] dryPulses = {{3320.496894, 3166.770186}, {3351.242236, 3259.006211, 3197.515528}};
    for (double[] pulse : dryPulses) {
      gate.update(0, false, false, 3350, 3350);
      for (int i = 0; i < pulse.length; i++) {
        gate.update(i * .08, true, i == 0, pulse[i], 3350);
        assertFalse("Near-target fluctuations must not count as departures", gate.ballLeft());
      }
      assertEquals(0, gate.ballsLeft());
    }
  }

  @Test public void volleyWaitsPastOvershootUntilTheBallLoadsTheWheels() {
    OneBallFeedGate gate = volleyGate();
    gate.update(0, true, true, 3289.819876, 3350);
    gate.update(.10, true, false, 3843.167702, 3350);
    gate.update(.14, true, false, 3689.440994, 3350);
    assertFalse("Falling from an overshoot is not a ball", gate.ballLeft());
    gate.update(.22, true, false, 2521.118012, 3350);
    assertTrue(gate.ballLeft());
    assertEquals(1, gate.ballsLeft());
  }

  @Test public void volleyAcceptsTheSmallerRecordedLoadedDip() {
    OneBallFeedGate gate = volleyGate();
    gate.update(0, true, true, 3289.819876, 3350);
    gate.update(.10, true, false, 3013.043478, 3350);
    assertTrue(gate.ballLeft()); // A 277 RPM fall: a ball under speed control (the old 300 waited for more)
  }

  @Test public void volleyCountsTheShallowBallsSpeedControlRecoversFrom() {
    // 12 Nov, 213719: balls 1 and 3 fell 216 and 277 RPM from the peak, and the 300 rule missed both
    double[][] balls = {{3536, 3474, 3320}, {3536, 3505, 3320, 3259}};
    for (double[] feed : balls) {
      OneBallFeedGate gate = volleyGate();
      gate.update(0, false, false, 3500, 3504);
      gate.update(.10, true, true, feed[0], 3504);
      for (int i = 1; i < feed.length; i++) { gate.update(.10 + i * .06, true, false, feed[i], 3504); }
      assertEquals(1, gate.ballsLeft());
    }
  }

  @Test public void volleyIgnoresTheWheelsWanderingAtRest() {
    // 12 Nov, 213719: 3443-3566 RPM around a 3500 target between balls
    OneBallFeedGate gate = volleyGate();
    double[] wander = {3566, 3443, 3505, 3566, 3443, 3474, 3536, 3443};
    for (int i = 0; i < wander.length; i++) { gate.update(i * .05, true, true, wander[i], 3500); }
    assertEquals(0, gate.ballsLeft());
  }

  @Test public void shallowFluctuationsCannotCompleteAFourBallTrial() {
    OneBallFeedGate gate = volleyGate();
    VolleyTrial trial = new VolleyTrial();
    trial.input(0, false, true, false);
    trial.input(.1, true, true, false);
    double[][] samples = {
        {.2, 3413}, {.3, 3812}, {.4, 3536}, {.5, 2952},
        {.8, 3382}, {.9, 3935}, {1, 3382}, {1.1, 2952},
        {1.4, 3320}, {1.5, 3167}, {1.6, 3351}, {1.7, 3259}, {1.8, 3198}};
    for (double[] sample : samples) {
      boolean ready = Math.abs(sample[1] - 3350) <= 75;
      gate.update(sample[0], true, ready, sample[1], 3350);
      trial.observe(gate.feedStarted(), gate.ballLeft());
    }
    assertEquals(2, trial.balls());
    assertTrue(trial.active());
    assertFalse(trial.takeFinished());
    gate.update(1.9, false, false, 3350, 3350);
    assertFalse(gate.ballLeft());
  }

  @Test public void confirmedVolleyStillStopsAfterFourLoadedDepartures() {
    OneBallFeedGate gate = volleyGate();
    VolleyTrial trial = new VolleyTrial();
    trial.input(0, false, true, false);
    trial.input(.1, true, true, false);
    for (int i = 0; i < 4; i++) {
      double now = .2 + i;
      gate.update(now, true, true, 3350, 3350);
      trial.observe(gate.feedStarted(), gate.ballLeft());
      gate.update(now + .1, true, false, 2900, 3350);
      trial.observe(gate.feedStarted(), gate.ballLeft());
    }
    assertEquals(4, trial.balls());
    assertFalse(trial.active());
    assertTrue(trial.takeFinished());
  }

  @Test public void volleyCannotImmediatelyRefeedTheSameRecoveryDip() {
    OneBallFeedGate gate = volleyGate();
    gate.update(0, true, true, 3350, 3350);
    gate.update(.1, true, false, 2900, 3350);
    assertTrue(gate.ballLeft());
    assertFalse(gate.update(.15, true, true, 3350, 3350));
    assertFalse(gate.update(.29, true, true, 3350, 3350));
    assertTrue(gate.update(.31, true, true, 3350, 3350));
  }

  @Test public void volleyStartsTheNextBallWhileTheWheelsClimbBack() {
    OneBallFeedGate gate = new OneBallFeedGate(0, .4, .3, .2, .05, true);
    gate.setLeadBelowRpm(150);
    assertTrue(gate.update(0, true, true, 3500, 3500, false, false, true));
    gate.update(.10, true, false, 3300, 3500, false, false, true); // Ball out: 200 from the peak, 200 under target
    assertTrue(gate.ballLeft());
    assertFalse("Still falling", gate.update(.16, true, false, 3150, 3500, false, false, true));
    assertFalse("Rising, but further under than the lead", gate.update(.20, true, false, 3300, 3500, false, false, true));
    assertTrue("Back within 150 and rising: the next ball starts before the band",
        gate.update(.24, true, false, 3380, 3500, false, false, true));
  }

  @Test public void volleyLeadWaitsForTheAimAndNeverLeadsTheFirstBall() {
    OneBallFeedGate gate = new OneBallFeedGate(0, .4, .3, .2, .05, true);
    gate.setLeadBelowRpm(150);
    assertFalse("First ball: the band", gate.update(0, true, false, 3400, 3500, false, false, true));
    gate.update(.02, true, true, 3500, 3500, false, false, true);
    gate.update(.10, true, false, 3300, 3500, false, false, true);
    gate.update(.16, true, false, 3150, 3500, false, false, true);
    assertFalse("Heading not ready", gate.update(.24, true, false, 3400, 3500, false, false, false));
    assertTrue(gate.update(.28, true, false, 3420, 3500, false, false, true));
  }

  @Test public void volleyCountsABallFalling170FromThePeak() {
    OneBallFeedGate gate = volleyGate();
    gate.update(0, true, true, 3500, 3500);
    gate.update(.10, true, false, 3325, 3500); // 175 down: the 13 Nov balls the 200 rule missed
    assertTrue(gate.ballLeft());
  }

  private OneBallFeedGate volleyGate() {
    return new OneBallFeedGate(0, .4, .3, .2, .2, true);
  }

}
