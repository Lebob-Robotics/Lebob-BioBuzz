package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotMarkingTest {
  @Test
  public void tapXIsIn() {
    MarkKeys k = new MarkKeys();
    assertNull(k.update(true, false, false, false, false, false));
    assertTrue(k.isMarking());
    assertEquals(BallMark.IN, k.update(false, false, false, false, false, false));
  }

  @Test
  public void holdXAndTapTheDpadForTheMissThenLetGo() {
    MarkKeys k = new MarkKeys();
    k.update(true, false, false, false, false, false);
    k.update(true, false, false, true, false, false); // Down: short
    k.update(true, false, false, false, false, false);
    k.update(true, false, false, false, true, false); // Left
    assertEquals("SHORT+LEFT", k.pending());
    BallMark m = k.update(false, false, false, false, false, false);
    assertEquals("SHORT+LEFT", m.toString());
    assertTrue(m.shortMiss && m.left && !m.in);
    // And the next tap starts clean
    k.update(true, false, false, false, false, false);
    assertEquals(BallMark.IN, k.update(false, false, false, false, false, false));
  }

  @Test
  public void aIsBouncedOutOnceAPress() {
    MarkKeys k = new MarkKeys();
    assertEquals(BallMark.BOUNCE, k.update(false, true, false, false, false, false));
    assertNull(k.update(false, true, false, false, false, false));
  }

  @Test
  public void markNames() {
    assertEquals("LONG+RIGHT", BallMark.miss(false, true, false, true).toString());
    assertEquals("MISS", BallMark.miss(false, false, false, false).toString());
    assertEquals("IN", BallMark.IN.toString());
    assertEquals("BOUNCE", BallMark.BOUNCE.toString());
  }

  @Test
  public void trimTurnsAwayFromTheMissAndHalvesWhenItSwapsSides() {
    AimTrimSearch t = new AimTrimSearch(1.0, 0.25);
    assertEquals(-1.0, t.record(BallMark.miss(false, false, true, false)), 1e-9); // Left: aim right
    assertEquals(-1.0, t.record(BallMark.miss(false, false, true, false)), 1e-9);
    assertEquals(0.5, t.record(BallMark.miss(false, false, false, true)), 1e-9); // Right now: half back
    assertEquals(-1.5, t.trimDeg(), 1e-9);
    assertEquals(0, t.record(BallMark.IN), 1e-9);
    assertEquals(0, t.record(BallMark.miss(true, false, false, false)), 1e-9); // Short says nothing about aim
    t.set(2.0);
    assertEquals(2.0, t.trimDeg(), 1e-9);
    assertEquals(1.0, t.stepDeg(), 1e-9);
  }

  @Test
  public void aimStepGrowsWhileMissesKeepGoingOneWay() {
    // 7 Nov, 080156: halved to 0.25 deg, 8 balls left in a row moved the aim only 1.75 deg
    AimTrimSearch t = new AimTrimSearch(1.0, 0.25);
    BallMark left = BallMark.miss(false, false, true, false);
    BallMark right = BallMark.miss(false, false, false, true);
    t.record(left);
    t.record(right);
    t.record(left);
    t.record(right); // Step down to 0.25
    assertEquals(0.25, t.stepDeg(), 1e-9);
    t.record(left);
    t.record(left);
    t.record(left);
    t.record(left);
    t.record(left);
    assertEquals(2.0, t.stepDeg(), 1e-9);
  }

  @Test
  public void replayedBallsMoveOnFromTheTrimTheyWereFiredWith() {
    AimTrimSearch t = new AimTrimSearch(1.0, 0.25);
    t.record(BallMark.miss(false, false, true, false), 3.0);
    assertEquals(2.0, t.trimDeg(), 1e-9);
  }
}
