package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class PoseHistoryTest {
  private static final double TOLERANCE = 1e-9;

  @Test
  public void blendsBetweenTheTwoSamplesAroundATime() {
    PoseHistory history = new PoseHistory(100);
    history.add(1000, new Pose(0, 0, 0));
    history.add(2000, new Pose(2, 4, 1));
    Pose halfway = history.sample(1500);
    assertNotNull(halfway);
    assertEquals(1, halfway.xM, TOLERANCE);
    assertEquals(2, halfway.yM, TOLERANCE);
    assertEquals(0.5, halfway.headingRad, TOLERANCE);
    assertEquals(2, history.sample(2000).xM, TOLERANCE);
    assertEquals(2, history.size());
  }

  @Test
  public void headingTakesTheShortWayRoundPi() {
    PoseHistory history = new PoseHistory(100);
    history.add(0, new Pose(0, 0, Math.PI - 0.1));
    history.add(100, new Pose(0, 0, -Math.PI + 0.1));
    assertEquals(Math.PI, Math.abs(history.sample(50).headingRad), TOLERANCE); // Not 0
    double quarter = history.sample(25).headingRad;
    assertEquals(0, ShootOnTheMove.normalizeAngle(quarter - (Math.PI - 0.05)), TOLERANCE);
  }

  @Test
  public void givesNothingFarOutsideTheStoredTimes() {
    PoseHistory history = new PoseHistory(100);
    assertNull(history.sample(0));
    for (int i = 0; i < 5; i++) {
      history.add(1000 + i * 100L, new Pose(i, 0, 0));
    }
    assertNull(history.sample(500));
    assertNull(history.sample(5000));
    assertEquals(0, history.sample(950).xM, TOLERANCE); // Within one sample gap of an end gives that end
    assertEquals(4, history.sample(1450).xM, TOLERANCE);
    assertNull(history.sample(899));
  }

  @Test
  public void theOldestSampleIsOverwrittenFirst() {
    PoseHistory history = new PoseHistory(3);
    for (int i = 0; i < 5; i++) {
      history.add(i * 10L, new Pose(i, 0, 0));
    }
    assertEquals(3, history.size());
    assertEquals(2, history.sample(20).xM, TOLERANCE); // Oldest kept
    assertEquals(3, history.sample(30).xM, TOLERANCE);
    assertEquals(3.5, history.sample(35).xM, TOLERANCE);
    assertNull(history.sample(0));
  }

  @Test
  public void shiftMovesEverySampleAndClearEmptiesIt() {
    PoseHistory history = new PoseHistory(100);
    history.add(0, new Pose(0, 0, 0));
    history.add(10, new Pose(1, 1, 0));
    history.shift(0.5, -0.25);
    assertEquals(0.5, history.sample(0).xM, TOLERANCE);
    assertEquals(-0.25, history.sample(0).yM, TOLERANCE);
    assertEquals(1.5, history.sample(10).xM, TOLERANCE);
    assertEquals(0.75, history.sample(10).yM, TOLERANCE);
    history.clear();
    assertEquals(0, history.size());
    assertNull(history.sample(0));
  }
}
