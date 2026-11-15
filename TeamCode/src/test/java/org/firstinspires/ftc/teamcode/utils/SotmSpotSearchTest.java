package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

public class SotmSpotSearchTest {
  /**
   * A shooter that scores with the range trim inside [lo, hi] and the aim within 0.3 deg of {@code aimDeg}. More
   * launch angle is shorter; more RPM is longer. More heading trim (anticlockwise) puts the ball left.
   */
  private static BallMark shoot(SotmSpotSearch s, double lo, double hi, double aimDeg, Random noise) {
    double r = s.rangeTrim() + (noise == null ? 0 : noise.nextGaussian() * (s.rpm ? 10 : 0.15));
    double h = s.headingTrimDeg() + (noise == null ? 0 : noise.nextGaussian() * 0.08);
    boolean below = r < lo;
    boolean above = r > hi;
    boolean shortMiss = s.rpm ? below : above;
    boolean longMiss = s.rpm ? above : below;
    boolean left = h > aimDeg + 0.3;
    boolean right = h < aimDeg - 0.3;
    if (!shortMiss && !longMiss && !left && !right) {
      return BallMark.parse("IN");
    }
    return BallMark.miss(shortMiss, longMiss, left, right);
  }

  private static SotmSpotSearch run(SotmSpotSearch s, double lo, double hi, double aimDeg, Random noise) {
    for (int i = 0; i < 60 && !s.done(); i++) {
      double fired = s.rangeTrim();
      s.record(fired, shoot(s, lo, hi, aimDeg, noise));
    }
    return s;
  }

  private static SotmSpotSearch hood(double startAim) {
    return new SotmSpotSearch(false, -10, 10, startAim, 2.0, 0.5, 3, 1.0, 0.25);
  }

  @Test
  public void findsTheHoodRangeAndTheAimFromMarksAlone() {
    SotmSpotSearch s = run(hood(0.4), 1.0, 3.0, -0.8, null);
    assertTrue("done in " + s.balls(), s.done());
    double[] r = s.range();
    // Each edge lies between a ball in and a miss, so it is known to the resolution, 0.5
    assertEquals(1.0, r[0], 0.5 + 1e-9);
    assertEquals(3.0, r[1], 0.5 + 1e-9);
    assertEquals(2.0, s.centre(), 0.5);
    assertEquals(-0.8, s.headingTrimDeg(), 0.3);
    assertTrue("balls " + s.balls(), s.balls() <= 20);
  }

  @Test
  public void findsItWithNoisyShotsToo() {
    int good = 0;
    for (int seed = 0; seed < 20; seed++) {
      SotmSpotSearch s = run(hood(0), -2.5, -0.5, 0.6, new Random(seed));
      if (s.done() && Math.abs(s.centre() + 1.5) <= 0.75 && Math.abs(s.headingTrimDeg() - 0.6) <= 0.4) {
        good++;
      }
    }
    assertTrue("good " + good + "/20", good >= 16);
  }

  @Test
  public void rpmSearchGoesTheOtherWay() {
    SotmSpotSearch s = run(new SotmSpotSearch(true, -800, 800, 0, 100, 25, 3, 1.0, 0.25), 100, 250, 0, null);
    assertTrue(s.done());
    assertEquals(175, s.centre(), 40);
  }

  @Test
  public void aChangedMarkReplaysEveryBall() {
    SotmSpotSearch a = hood(0);
    SotmSpotSearch b = hood(0);
    BallMark longLeft = BallMark.miss(false, true, true, false);
    BallMark shortRight = BallMark.miss(true, false, false, true);
    a.record(a.rangeTrim(), longLeft);
    b.record(b.rangeTrim(), longLeft);
    double fired = a.rangeTrim();
    a.record(fired, BallMark.parse("IN"));
    a.remark(shortRight);
    b.record(fired, shortRight);
    assertEquals(b.rangeTrim(), a.rangeTrim(), 1e-9);
    assertEquals(b.headingTrimDeg(), a.headingTrimDeg(), 1e-9);
    assertEquals(b.balls(), a.balls());
    assertEquals(b.scored(), a.scored());
  }

  @Test
  public void trimLimitsAlwaysIncludeTheTable() {
    SotmSpotSearch s = new SotmSpotSearch(false, 2, 5, 0, 2.0, 0.5, 3, 1.0, 0.25);
    assertEquals(0, s.minTrim, 1e-9);
    assertEquals(0, s.rangeTrim(), 1e-9);
  }
}
