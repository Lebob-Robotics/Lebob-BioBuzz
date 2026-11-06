package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotSearchTest {
  /** A shooter that scores between lo and hi launch degrees; lower launch goes further (the hooded shot). */
  private static BallMark hooded(double launch, double lo, double hi) {
    if (launch < lo) { return BallMark.miss(false, true, false, false); } // Too flat: long
    if (launch > hi) { return BallMark.miss(true, false, false, false); } // Too steep: short
    return BallMark.IN;
  }

  private static ShotSearch hoodSearch(double start) {
    return new ShotSearch(start, -1, 2.0, 0.5, 0.1, 55, 99, 3);
  }

  @Test
  public void findsBothEdgesOfTheScoringRangeFromInsideIt() {
    ShotSearch s = hoodSearch(65);
    int balls = 0;
    while (!s.hasRange() && balls < 30) {
      double launch = s.next();
      s.record(launch, hooded(launch, 63.3, 66.1));
      balls++;
    }
    assertTrue("took " + balls, balls <= 10);
    double[] r = s.range();
    assertEquals(63.3, r[0], 0.5);
    assertEquals(66.1, r[1], 0.5);
    assertEquals((r[0] + r[1]) / 2, s.next(), 0.11); // Confirms at the middle
    assertEquals(ShotSearch.Phase.CONFIRM, s.phase());
  }

  @Test
  public void walksTowardTheRangeFromAShortStartThenFindsIt() {
    ShotSearch s = hoodSearch(72); // Steep: every ball short
    double first = s.next();
    s.record(first, hooded(first, 63, 65));
    assertTrue("a short ball flattens the shot", s.next() < first);
    int balls = 1;
    while (!s.hasRange() && balls < 30) {
      double launch = s.next();
      s.record(launch, hooded(launch, 63, 65));
      balls++;
    }
    assertTrue("took " + balls, balls <= 14);
    assertEquals(63, s.range()[0], 0.5);
    assertEquals(65, s.range()[1], 0.5);
  }

  @Test
  public void aRangeNarrowerThanAStepIsStillFoundByHalving() {
    ShotSearch s = hoodSearch(70);
    int balls = 0;
    while (!s.hasRange() && balls < 40) {
      double launch = s.next();
      s.record(launch, hooded(launch, 66.2, 66.9));
      balls++;
    }
    assertTrue(s.hasRange());
    assertTrue(s.centre() >= 66.2 - 0.3 && s.centre() <= 66.9 + 0.3);
  }

  @Test
  public void confirmBallsThenDone() {
    ShotSearch s = new ShotSearch(65, -1, 2.0, 0.5, 0.1, 55, 99, 2);
    while (!s.hasRange()) {
      double launch = s.next();
      s.record(launch, hooded(launch, 64, 66));
    }
    s.record(s.next(), BallMark.IN);
    assertEquals(ShotSearch.Phase.CONFIRM, s.phase());
    s.record(s.next(), BallMark.miss(true, false, false, false));
    assertEquals(ShotSearch.Phase.DONE, s.phase());
    assertEquals(2, s.confirmFired());
    assertEquals(1, s.confirmScored());
  }

  @Test
  public void aSideOnlyMissFiresTheSameValueAgain() {
    ShotSearch s = hoodSearch(65);
    s.record(65, BallMark.miss(false, false, true, false));
    assertEquals(65, s.next(), 1e-9);
    s.record(65, BallMark.miss(true, true, false, false)); // Short and long: no range information
    assertEquals(65, s.next(), 1e-9);
    assertEquals(ShotSearch.Phase.FIND, s.phase());
  }

  @Test
  public void aBounceCountsOnTheSideOfTheScoringBallsItWasFiredOn() {
    ShotSearch s = hoodSearch(65);
    s.record(65, BallMark.IN);
    double probe = s.next(); // Out toward the long edge: flatter
    assertTrue(probe < 65);
    s.record(probe, BallMark.BOUNCE);
    assertEquals(ShotSearch.Phase.LONG_EDGE, s.phase());
    assertTrue("bisects between the scoring ball and the bounce", s.next() < 65 && s.next() > probe);
  }

  @Test
  public void aBallThatScoresPastAnEdgeForgetsThatEdgesMiss() {
    ShotSearch s = hoodSearch(65);
    s.record(65, BallMark.IN);
    s.record(63, BallMark.miss(false, true, false, false)); // Long at 63
    s.record(62, BallMark.IN); // But 62, flatter still, scored: the long ball was noise
    assertEquals(ShotSearch.Phase.LONG_EDGE, s.phase());
    assertTrue(s.next() < 62);
  }

  @Test
  public void stopsAtTheSettingsLimitAndTakesItAsTheEdge() {
    // Backward shot: raising the launch angle sends it further, and 99 is the hood's stop
    ShotSearch s = new ShotSearch(98, 1, 2.0, 0.5, 0.1, 85, 99, 0);
    s.record(98, BallMark.IN);
    assertEquals(99, s.next(), 1e-9);
    s.record(99, BallMark.IN);
    assertEquals(ShotSearch.Phase.SHORT_EDGE, s.phase());
    assertTrue(s.next() < 98);
    s.record(s.next(), BallMark.miss(true, false, false, false));
    while (!s.hasRange()) {
      double v = s.next();
      s.record(v, v >= 97 ? BallMark.IN : BallMark.miss(true, false, false, false));
    }
    assertEquals(99, s.range()[1], 1e-9);
    assertEquals(97, s.range()[0], 0.5);
    assertEquals(ShotSearch.Phase.DONE, s.phase());
  }

  @Test
  public void aMissPastTheScoringBallsBoundsThatEdgeWhateverItWasCalled() {
    // Back G0, 6 Nov: 90-94 in, then short from 96 up; searched as if higher went further, it fired 98 nineteen times
    ShotSearch s = new ShotSearch(90, 1, 2.0, 0.5, 0.1, 86, 99, 3);
    s.record(90, BallMark.IN);
    s.record(92, BallMark.IN);
    s.record(94, BallMark.IN);
    assertEquals(96, s.next(), 1e-9);
    s.record(96, BallMark.miss(true, false, false, false));
    assertEquals(ShotSearch.Phase.LONG_EDGE, s.phase());
    assertEquals(95, s.next(), 1e-9);
    assertEquals(1, s.contrary());
  }

  @Test
  public void theBackwardShotMarkedShortSteepensIt() {
    // A higher backward launch angle is flatter and comes in under the rim, so short lowers it
    ShotSearch s = new ShotSearch(94, -1, 2.0, 0.5, 0.1, 86, 99, 3);
    s.record(94, BallMark.miss(true, false, false, false));
    assertEquals(92, s.next(), 1e-9);
    s.record(92, BallMark.IN);
    assertEquals(90, s.next(), 1e-9); // Out toward the steep edge
    s.record(90, BallMark.miss(false, true, false, false)); // Over the back
    assertEquals(0, s.contrary());
    assertEquals(91, s.next(), 1e-9);
  }

  @Test
  public void searchesRpmWithMoreRpmGoingFurther() {
    ShotSearch s = new ShotSearch(3400, 1, 100, 25, 10, 2500, 4500, 3);
    s.record(3400, BallMark.miss(true, false, false, false));
    assertEquals(3500, s.next(), 1e-9);
  }

  @Test
  public void rangeIsNaNUntilABallGoesIn() {
    ShotSearch s = hoodSearch(65);
    assertTrue(Double.isNaN(s.range()[0]));
    assertTrue(Double.isNaN(s.centre()));
  }
}
