package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.auto.AutoSequence.Step;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.junit.After;
import org.junit.Test;

public class AutoSequenceTest {
  private static final double DT = 0.02;
  private final AutoSequence seq = new AutoSequence(0);
  private double t = 0;

  @After
  public void secondFlowerOffAgain() {
    AutoSequence.SECOND_FLOWER = false;
  }

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

  /** Every step arrives at once; the robot is empty, and full at a Flower: the quickest way through. */
  private void quickTo(Step target) {
    double from = t;
    while (seq.step() != target) {
      t += DT;
      seq.update(t, true, false, true, true);
      assertTrue("stuck in " + seq.step() + " before " + target, t - from < 30);
    }
  }

  /** Arrives at the shooting spot 0.5 s in and shoots 4. */
  private void volley(double startS) {
    runUntil(startS + 2.0, startS + 0.5, startS + 0.8, startS + 1.1, startS + 1.4, startS + 1.7);
  }

  @Test
  public void followsThePlan() {
    assertTrue(seq.takeNewStep());
    assertEquals(Step.SHOOT_PRELOAD, seq.step());
    assertFalse("not shooting on the way to the Hive", seq.shoot(0));
    assertFalse("nor pushing the full stack into the spinning wheels", seq.intake());
    runUntil(0.6, 0.5);
    assertTrue("stopped at the Hive: shooting", seq.shoot(t));
    assertTrue("the intake pushes the 4th ball up while shooting", seq.intake());
    runUntil(2.0, 0.5, 0.8, 1.1, 1.4, 1.7);
    assertEquals(4, seq.ballsShot());
    assertEquals(Step.DRIVE_TO_GARDEN, seq.step());
    assertFalse(seq.shoot(t));
    assertFalse(seq.intake());

    for (Step s : new Step[] {Step.FACE_GARDEN, Step.ONTO_GARDEN_WALL, Step.SWEEP_GARDEN}) {
      double start = t;
      runUntil(start + 0.5, start + 0.4);
      assertEquals(s, seq.step());
      assertTrue(s + " intake", seq.intake());
      assertFalse(s + " no arm at the Garden", seq.armDown());
      assertFalse(s + " not shooting", seq.shoot(t));
    }
    double sweepStart = t;
    runUntil(sweepStart + 2.0, sweepStart + 1.9);
    assertEquals(Step.SHOOT_GARDEN, seq.step());
    assertFalse("not shooting on the way to the other Cell", seq.shoot(t));
    assertTrue("still drawing the Garden's balls in on the way", seq.intake());
    runUntil(t + 0.3, t + 0.2);
    assertTrue("stopped, shooting", seq.shoot(t));
    assertTrue("and still pushing the last balls up", seq.intake());
    volley(t);
    assertEquals(8, seq.ballsShot());

    assertEquals(Step.DRIVE_TO_WALL_FLOWER, seq.step());
    assertFalse(seq.shoot(t));
    double driveStart = t;
    runUntil(driveStart + 1.6, driveStart + 1.5);
    assertEquals(Step.LOWER_ARM_WALL, seq.step());
    assertFalse("arm still up while it turns to the Flower", seq.armDown());
    assertTrue(seq.isFacingStep());
    double lowerStart = t;
    runUntil(lowerStart + 0.3, lowerStart + 0.2);
    assertTrue("facing it: arm down", seq.armDown());
    runUntil(lowerStart + 0.2 + AutoSequence.ARM_MOVE_S + 2 * DT, lowerStart + 0.2);
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    assertTrue(seq.armDown());
    assertTrue(seq.intake());
    double intakeStart = t;
    runUntil(intakeStart + 0.5 + AutoSequence.FLOWER_DWELL_S - 2 * DT, intakeStart + 0.5);
    assertEquals("in the Flower for the last balls", Step.INTAKE_WALL_FLOWER, seq.step());
    runUntil(intakeStart + 0.5 + AutoSequence.FLOWER_DWELL_S + 2 * DT, intakeStart + 0.5);
    assertEquals(Step.BACK_FROM_WALL_FLOWER, seq.step());
    assertTrue("arm down backing out", seq.armDown());
    double backStart = t;
    runUntil(backStart + 0.8, backStart + 0.7);
    assertEquals(Step.RAISE_ARM_WALL, seq.step());
    assertFalse("raising", seq.armDown());
    runUntil(t + AutoSequence.ARM_MOVE_S + DT, Double.POSITIVE_INFINITY);
    assertEquals(Step.SHOOT_WALL_FLOWER, seq.step());
    volley(t);
    assertEquals(12, seq.ballsShot());
    assertEquals("no second Flower: straight to park", Step.DRIVE_TO_PARK, seq.step());
  }

  @Test
  public void afterThePreloadsEveryVolleyShootsTheOtherCell() {
    for (Cell first : Cell.values()) {
      Cell other = AutoSequence.opposite(first);
      assertEquals(first, AutoSequence.plannedCell(first, Step.SHOOT_PRELOAD));
      assertEquals("the preloads tipped it", other, AutoSequence.plannedCell(first, Step.SHOOT_GARDEN));
      assertEquals("the same Cell after the Flower", other, AutoSequence.plannedCell(first, Step.SHOOT_WALL_FLOWER));
      assertEquals(other, AutoSequence.plannedCell(first, Step.PARKED));
    }
  }

  @Test
  public void anEmptyFlowerIsLeftSoonAfterDrivingIn() {
    t = 1.0;
    quickTo(Step.INTAKE_WALL_FLOWER);
    double flowerStart = t;
    while (seq.step() == Step.INTAKE_WALL_FLOWER) {
      t += DT;
      seq.update(t, t - flowerStart >= 1.2, false, true, false); // In after 1.2 s, as 14 Nov; never full
    }
    assertEquals(Step.BACK_FROM_WALL_FLOWER, seq.step());
    assertEquals(1.2 + AutoSequence.FLOWER_DWELL_S, t - flowerStart, 2 * DT);
  }

  /**
   * The 14 Nov 205111 mock run's drive times (no balls: each volley ended 1.8 s after stopping), the drive to the
   * other Cell after the Garden longer. It must take the second Flower and park well before 30 s.
   */
  @Test
  public void aMockRunOfEmptyVolleysParksInTime() { // Each volley waits out EMPTY_S: the second Flower is skipped
    AutoSequence.SECOND_FLOWER = true;
    java.util.Map<Step, Double> drive = new java.util.EnumMap<>(Step.class);
    drive.put(Step.SHOOT_PRELOAD, 1.4);
    drive.put(Step.DRIVE_TO_GARDEN, 1.4);
    drive.put(Step.FACE_GARDEN, 0.1);
    drive.put(Step.ONTO_GARDEN_WALL, 0.7);
    drive.put(Step.SWEEP_GARDEN, 1.8);
    drive.put(Step.SHOOT_GARDEN, 2.4);
    drive.put(Step.DRIVE_TO_WALL_FLOWER, 1.4);
    drive.put(Step.LOWER_ARM_WALL, 0.3);
    drive.put(Step.INTAKE_WALL_FLOWER, 1.2);
    drive.put(Step.BACK_FROM_WALL_FLOWER, 0.8);
    drive.put(Step.SHOOT_WALL_FLOWER, 1.0);
    drive.put(Step.DRIVE_TO_REAR_FLOWER, 2.5);
    drive.put(Step.LOWER_ARM_REAR, 0.3);
    drive.put(Step.INTAKE_REAR_FLOWER, 1.2);
    drive.put(Step.BACK_FROM_REAR_FLOWER, 0.8);
    drive.put(Step.DRIVE_TO_PARK, 1.6);
    boolean rearFlower = false;
    Step step = null;
    double stepStart = 0;
    while (seq.step() != Step.PARKED) {
      if (seq.step() != step) {
        step = seq.step();
        stepStart = t;
      }
      rearFlower |= step == Step.INTAKE_REAR_FLOWER;
      t += DT;
      double driven = t - stepStart - drive.getOrDefault(step, 0.0);
      seq.update(t, driven >= 0, false, driven >= 1.2, false);
      assertTrue("still " + step + " at " + t, t < 30);
    }
    assertFalse("no time for the second Flower", rearFlower);
    assertTrue("parked by " + t, t < 28.0);
  }

  @Test
  public void inAFlowerItWigglesBothWaysFromFacingIt() {
    assertEquals("no jump going in", 0.0, AutoSequence.flowerWiggle(0)[0], 1e-9);
    double left = 0;
    double right = 0;
    for (double s = 0; s < AutoSequence.FLOWER_DWELL_S; s += 0.005) {
      double[] w = AutoSequence.flowerWiggle(s);
      left = Math.max(left, w[0]);
      right = Math.min(right, w[0]);
      double[] next = AutoSequence.flowerWiggle(s + 1e-4);
      assertEquals("rate at " + s, (next[0] - w[0]) / 1e-4, w[1], 0.01);
    }
    assertEquals(AutoSequence.WIGGLE_RAD, left, 1e-3);
    assertEquals(-AutoSequence.WIGGLE_RAD, right, 1e-3);
    assertTrue("a whole wiggle while it waits in it", AutoSequence.FLOWER_DWELL_S >= AutoSequence.WIGGLE_PERIOD_S);
  }

  @Test
  public void neverPicksUpBeforeShootingTheLastPickup() {
    boolean holding = true; // The preloads
    for (Step s : Step.values()) {
      if (s.shooting) {
        holding = false;
      }
      boolean picksUp = s == Step.SWEEP_GARDEN || s == Step.INTAKE_WALL_FLOWER || s == Step.INTAKE_REAR_FLOWER;
      if (picksUp) {
        assertFalse(s + " picks up still holding the last pickup", holding);
        holding = true;
      }
    }
  }

  @Test
  public void theRearFlowerIsTakenTheSameWayThenParksAndShoots() {
    AutoSequence.SECOND_FLOWER = true;
    t = 1.0;
    quickTo(Step.DRIVE_TO_REAR_FLOWER);
    for (Step expected : new Step[] {Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.RAISE_ARM_REAR, Step.DRIVE_TO_PARK, Step.PARKED}) {
      quickTo(expected);
    }
    assertFalse(seq.armDown());
    t += DT;
    seq.update(t, true, false, false);
    assertTrue("parked: shoots what it holds", seq.shoot(t));
  }

  @Test
  public void shootingLateAfterTheFirstFlowerSkipsTheSecond() {
    AutoSequence.SECOND_FLOWER = true;
    t = 12.0; // Running late
    quickTo(Step.SHOOT_WALL_FLOWER);
    t = AutoSequence.SKIP_SECOND_FLOWER_S;
    quickTo(Step.DRIVE_TO_PARK);
    assertTrue(seq.isSkippingSecondFlower());
  }

  @Test
  public void anyStepStillGoingAtTheParkDeadlineGoesToPark() {
    runUntil(AutoSequence.PARK_BY_S + DT, Double.POSITIVE_INFINITY); // Never reaches the Hive
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
  }

  @Test
  public void anEmptyRobotEndsItsVolleyOnceStopped() {
    t = 0.5;
    seq.update(t, false, false, true); // Driving: an empty feeder doesn't count yet
    t += AutoSequence.EMPTY_S + DT;
    seq.update(t, false, false, true);
    assertEquals(Step.SHOOT_PRELOAD, seq.step());
    t += DT;
    double arrived = t;
    seq.update(t, true, false, true);
    assertTrue(seq.shoot(t));
    while (seq.step() == Step.SHOOT_PRELOAD) {
      t += DT;
      seq.update(t, true, false, true);
    }
    assertEquals(Step.DRIVE_TO_GARDEN, seq.step());
    assertTrue("left once empty, not after the whole wait", t < arrived + AutoSequence.EMPTY_S + 3 * DT);
  }

  @Test
  public void aVolleyThatNeverFinishesIsGivenUpAfterTheWait() {
    runUntil(1.0, 1.0, 1.0);
    assertEquals(Step.SHOOT_PRELOAD, seq.step());
    runUntil(1.0 + AutoSequence.VOLLEY_WAIT_S - 2 * DT, 1.0);
    assertEquals(Step.SHOOT_PRELOAD, seq.step());
    runUntil(1.0 + AutoSequence.VOLLEY_WAIT_S + 2 * DT, 1.0);
    assertEquals(Step.DRIVE_TO_GARDEN, seq.step());
  }

  @Test
  public void neverFacingThePollenStillSweepsAfterTheWait() {
    t = 0.5;
    quickTo(Step.FACE_GARDEN);
    double faceStart = t;
    runUntil(faceStart + AutoSequence.FACE_GARDEN_MAX_S - 2 * DT, Double.POSITIVE_INFINITY);
    assertEquals(Step.FACE_GARDEN, seq.step());
    runUntil(faceStart + AutoSequence.FACE_GARDEN_MAX_S + 2 * DT, Double.POSITIVE_INFINITY);
    assertEquals(Step.ONTO_GARDEN_WALL, seq.step());
    runUntil(t + AutoSequence.ONTO_WALL_MAX_S + 2 * DT, Double.POSITIVE_INFINITY);
    assertEquals(Step.SWEEP_GARDEN, seq.step());
  }

  @Test
  public void neverFacingTheFlowerStillGoesInAfterTheWait() {
    t = 1.0;
    quickTo(Step.LOWER_ARM_WALL);
    double lowerStart = t;
    while (seq.step() == Step.LOWER_ARM_WALL) {
      t += DT;
      seq.update(t, false, false, true); // Never lines up
      assertTrue(t - lowerStart < AutoSequence.FACE_FLOWER_MAX_S + 2 * DT);
    }
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    assertTrue(seq.armDown());
  }

  @Test
  public void aFullRobotLeavesTheFlowerAtOnce() {
    t = 1.0;
    quickTo(Step.INTAKE_WALL_FLOWER);
    double flowerStart = t;
    t += DT;
    seq.update(t, true, false, false, false);
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    t += DT;
    seq.update(t, true, false, false, true);
    assertEquals(Step.BACK_FROM_WALL_FLOWER, seq.step());
    assertTrue(t - flowerStart < AutoSequence.FLOWER_INTAKE_S);
  }

  @Test
  public void theArmIsUpAgainBeforeTheParkDeadline() {
    double intakeS = AutoSequence.FLOWER_INTAKE_S;
    AutoSequence.FLOWER_INTAKE_S = 30; // A Flower that never runs out
    try {
      t = 12.0;
      quickTo(Step.INTAKE_WALL_FLOWER);
      t = AutoSequence.PARK_BY_S - AutoSequence.FLOWER_EXIT_S - 4 * DT;
      runUntil(t + 2 * DT, Double.POSITIVE_INFINITY);
      assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
      runUntil(t + 3 * DT, Double.POSITIVE_INFINITY);
      assertEquals(Step.BACK_FROM_WALL_FLOWER, seq.step());
      while (seq.step() != Step.SHOOT_WALL_FLOWER) {
        t += DT;
        seq.update(t, t > AutoSequence.PARK_BY_S - 1.0, false, true);
      }
      assertTrue("arm up by " + t, t <= AutoSequence.PARK_BY_S);
      assertFalse(seq.armDown());
    } finally {
      AutoSequence.FLOWER_INTAKE_S = intakeS;
    }
  }

  @Test
  public void atTheParkDeadlineBackingOutFinishesFirst() {
    t = 1.0;
    quickTo(Step.BACK_FROM_WALL_FLOWER);
    t = AutoSequence.PARK_BY_S + DT; // Long after: backing out has timed out
    seq.update(t, false, false, true);
    assertEquals("raises the arm before it parks", Step.RAISE_ARM_WALL, seq.step());
    while (seq.step() != Step.DRIVE_TO_PARK) {
      t += DT;
      seq.update(t, true, false, true);
      assertTrue("at " + seq.step(), t < AutoSequence.PARK_BY_S + 1.0);
    }
  }

  @Test
  public void parkedItKeepsShootingToTheEnd() {
    while (seq.step() != Step.PARKED) {
      t += DT;
      seq.update(t, true, false, true, false);
      assertTrue(t < 30);
    }
    t += DT;
    seq.update(t, true, false, true, false);
    for (double end = t + 3; t < end; t += DT) {
      seq.update(t, true, false, true, false);
    }
    assertTrue("still shooting long after the feeder found nothing", seq.shoot(t));
    assertTrue("pushing up what's left in the intake", seq.intake());
  }
}
