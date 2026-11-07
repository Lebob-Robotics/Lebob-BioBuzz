package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class BackwardShotSearchTest {
  private static final BallMark SHORT = BallMark.miss(true, false, false, false);
  private static final BallMark LONG = BallMark.miss(false, true, false, false);

  private static BackwardShotSearch search(double launch, double rpm) {
    return new BackwardShotSearch(launch, rpm, 2.0, 90, 100, 25, 2500, 4500, 3);
  }

  @Test
  public void searchesTheRpmAtTheTablesAngle() {
    BackwardShotSearch s = search(94.5, 3280);
    assertEquals(3280, s.rpm().next(), 1e-9);
    s.record(3280, SHORT);
    assertEquals(3380, s.rpm().next(), 1e-9);
    assertEquals(94.5, s.launchDeg(), 1e-9);
    assertEquals(0, s.steepened());
  }

  @Test
  public void aShortAndALongWithNothingBetweenSteepensTheHood() {
    // Shot Tuning 085918, 7 Nov, back G1 at 98.3 degrees: the bottom of the basket up to 4030 rpm, over the top from 4050
    BackwardShotSearch s = search(98.3, 3880);
    s.record(3880, SHORT);
    s.record(3980, SHORT);
    s.record(4080, LONG);
    assertEquals(98.3, s.launchDeg(), 1e-9); // 100 rpm apart: the window may still be between
    s.record(4030, SHORT);
    assertEquals(98.3, s.launchDeg(), 1e-9);
    s.record(4060, LONG);
    assertEquals(98.3, s.launchDeg(), 1e-9);
    s.record(4040, SHORT); // 20 rpm from the long with none in: no window at this angle
    assertEquals(96.3, s.launchDeg(), 1e-9);
    assertEquals(1, s.steepened());
    assertEquals(4050, s.rpm().next(), 1e-9);
    assertEquals(0, s.rpm().balls());
  }

  @Test
  public void aWindowKeepsTheAngle() {
    BackwardShotSearch s = search(94.5, 3280);
    s.record(3280, SHORT);
    s.record(3380, BallMark.IN);
    s.record(3480, LONG);
    s.record(3430, LONG);
    s.record(3400, LONG);
    s.record(3300, SHORT);
    s.record(3340, SHORT);
    s.record(3360, SHORT);
    assertEquals(94.5, s.launchDeg(), 1e-9);
    assertEquals(ShotSearch.Phase.CONFIRM, s.rpm().phase());
  }

  @Test
  public void stopsSteepeningAtStraightUp() {
    BackwardShotSearch s = search(91, 3300);
    s.record(3300, SHORT);
    s.record(3310, LONG);
    assertEquals(90, s.launchDeg(), 1e-9);
    s.record(3305, SHORT);
    s.record(3315, LONG);
    assertEquals(90, s.launchDeg(), 1e-9);
    assertEquals(1, s.steepened());
  }
}
