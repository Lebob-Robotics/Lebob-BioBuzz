package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.auto.AutoSequence.Step;
import org.junit.Test;

public class AutoSequenceTest {
  private static final double DT = 0.02;
  private final AutoSequence seq = new AutoSequence(0);
  private double t = 0;

  /**
   * Runs loops until the given time, a ball out at each of ballTimes. The path is done from arriveS on, until the
   * step changes: the next step's path then starts.
   */
  private void runUntil(double endS, double arriveS, double... ballTimes) {
    Step startStep = seq.step();
    while (t < endS - 1e-9) {
      t += DT;
      boolean ball = false;
      for (double b : ballTimes) {
        if (Math.abs(t - b) < DT / 2) {
          ball = true;
        }
      }
      seq.update(t, t >= arriveS && seq.step() == startStep, ball, false);
    }
  }

  @Test
  public void followsThePlanAndShootsEachVolleyWhileDriving() {
    assertTrue(seq.takeNewStep());
    assertEquals(Step.DRIVE_TO_GARDEN, seq.step());
    assertTrue(seq.shoot(0));
    assertFalse(seq.intake());
    runUntil(3.0, 2.5, 0.8, 1.2, 1.6, 2.0);
    assertFalse("volley done", seq.shoot(t));
    assertEquals(Step.SWEEP_GARDEN, seq.step());
    assertTrue(seq.takeNewStep());
    assertTrue(seq.intake());
    double sweepStart = t;
    runUntil(sweepStart + 2.4, sweepStart + 2.3);
    assertEquals(Step.DRIVE_TO_WALL_FLOWER, seq.step());
    double driveStart = t;
    runUntil(driveStart + 3.0, driveStart + 2.0, driveStart + 1.0, driveStart + 1.4, driveStart + 1.8, driveStart + 2.2);
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    double intakeStart = t;
    runUntil(intakeStart + AutoSequence.FLOWER_INTAKE_S + 0.1, intakeStart + 0.5);
    assertTrue("left the first Flower before 16 s", t < AutoSequence.SKIP_SECOND_FLOWER_S);
    assertEquals(Step.DRIVE_TO_REAR_FLOWER, seq.step());
    assertEquals(8, seq.ballsShot());
  }

  @Test
  public void stillAtTheFirstFlowerAtSixteenSecondsSkipsToPark() {
    t = 12.0; // Running late
    while (seq.step() != Step.INTAKE_WALL_FLOWER) {
      t += DT;
      seq.update(t, true, false, true); // Arrives at once and empty
      assertTrue("stuck in " + seq.step(), t < 16);
    }
    runUntil(15.9, Double.POSITIVE_INFINITY);
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    runUntil(17.5, Double.POSITIVE_INFINITY);
    assertTrue(seq.isSkippingSecondFlower());
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
  }

  @Test
  public void anyStepStillGoingAtTheParkDeadlineGoesToPark() {
    runUntil(AutoSequence.PARK_BY_S + DT, Double.POSITIVE_INFINITY); // Never arrives at the Garden
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
    assertTrue("still shooting on the way", seq.shoot(t));
  }

  @Test
  public void anEmptyRobotEndsItsVolley() {
    t = 0.5;
    seq.update(t, false, false, true);
    assertTrue(seq.shoot(t));
    t += AutoSequence.EMPTY_S + DT;
    seq.update(t, false, false, true);
    assertFalse(seq.shoot(t));
    t += DT;
    seq.update(t, true, false, true);
    assertEquals("arrived and empty: on to the sweep", Step.SWEEP_GARDEN, seq.step());
  }

  @Test
  public void aVolleyThatNeverFinishesIsGivenUpAfterTheWait() {
    runUntil(1.0, 1.0, 0.5);
    assertEquals(Step.DRIVE_TO_GARDEN, seq.step());
    runUntil(1.0 + AutoSequence.VOLLEY_WAIT_S + 3 * DT, 1.0);
    assertEquals(Step.SWEEP_GARDEN, seq.step());
  }

  @Test
  public void parkedKeepsShootingWhatIsLeft() {
    runUntil(AutoSequence.PARK_BY_S + DT, Double.POSITIVE_INFINITY);
    double parkStart = t;
    runUntil(parkStart + 1.0, parkStart + 0.5);
    runUntil(parkStart + 3.0, parkStart + 0.5);
    assertEquals(Step.PARKED, seq.step());
    assertTrue(seq.shoot(t));
  }
}
