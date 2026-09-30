package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.subsystems.IndexerStallGuard.Phase;
import org.junit.Before;
import org.junit.Test;

/** Currents and speeds are from the prototype's two-ball jam on 30 Sep. */
public class IndexerStallGuardTest {
  private static final double FEED = 1.0;
  private static final double UNJAM = -0.6;
  private static final double STEP_S = 0.05;

  private IndexerStallGuard guard;
  private double now;

  @Before
  public void setUp() {
    guard = new IndexerStallGuard(FEED, UNJAM);
    now = 0;
    guard.start(now);
  }

  /** Feeds the same reading for a duration, returning the last power. */
  private double run(double seconds, double amps, double velocity) {
    double power = guard.getPower();
    for (double end = now + seconds; now < end - 1e-9;) {
      now += STEP_S;
      power = guard.update(now, amps, velocity);
    }
    return power;
  }

  @Test
  public void feedsWhileRunningFreely() {
    assertEquals(FEED, run(2.0, 1.7, 2200), 0);
    assertEquals(Phase.FEEDING, guard.getPhase());
  }

  @Test
  public void heavyLoadThatStillMovesIsNotAStall() {
    assertEquals(FEED, run(2.0, 8.7, 460), 0);
    assertEquals(Phase.FEEDING, guard.getPhase());
  }

  @Test
  public void briefStallIsIgnored() {
    run(0.2, 10.5, 0);
    assertEquals(FEED, run(1.0, 7.5, 800), 0);
    assertEquals(0, guard.getUnjamAttempts());
  }

  @Test
  public void sustainedStallBacksOffThenResumes() {
    assertEquals(UNJAM, run(0.4, 10.5, 0), 0);
    assertEquals(Phase.UNJAMMING, guard.getPhase());
    assertEquals(1, guard.getUnjamAttempts());

    assertEquals(FEED, run(IndexerStallGuard.UNJAM_TIME_S, 3.0, -300), 0);
    assertEquals(Phase.FEEDING, guard.getPhase());
  }

  @Test
  public void givesUpAfterMaxAttemptsAndStaysOff() {
    for (int i = 0; i < IndexerStallGuard.MAX_UNJAM_ATTEMPTS; i++) {
      run(0.4, 10.5, 0);
      run(IndexerStallGuard.UNJAM_TIME_S, 3.0, -300);
    }
    assertEquals(0.0, run(0.4, 10.5, 0), 0);
    assertEquals(Phase.JAMMED, guard.getPhase());

    // Stays off even once the motor goes quiet, until a new feed starts.
    assertEquals(0.0, run(1.0, 0.0, 0), 0);
    guard.start(now);
    assertEquals(Phase.FEEDING, guard.getPhase());
    assertEquals(0, guard.getUnjamAttempts());
    assertEquals(FEED, guard.getPower(), 0);
  }
}
