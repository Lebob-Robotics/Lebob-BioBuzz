package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.CornerSequence.Step;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.junit.Test;

public class CornerSequenceTest {
  private static final double DT = 0.02;
  private final CornerSequence seq = new CornerSequence(0);
  private double t = 0;

  /** Every drive arrives 0.5 s after its step starts; the robot is empty (so volleys end), full at the Flower. */
  private void runTo(Step target, double limitS) {
    double stepStart = t;
    Step was = seq.step();
    while (seq.step() != target) {
      t += DT;
      if (seq.step() != was) {
        was = seq.step();
        stepStart = t;
      }
      seq.update(t, t - stepStart >= 0.5, false, true, true);
      assertTrue("stuck in " + seq.step() + " before " + target + " at " + t, t < limitS);
    }
  }

  @Test
  public void runsThePlanInOrderAndWaitsBackInTheCorner() {
    seq.takeNewStep();
    assertEquals(Step.SHOOT_PRELOAD, seq.step());
    runTo(Step.FACE_WALL, 5);
    runTo(Step.BACK_TO_CORNER, 6);
    runTo(Step.WAIT_IN_CORNER, 7);
    double waitFrom = t;
    runTo(Step.DRIVE_TO_FLOWER, 20);
    assertEquals(CornerSequence.WAIT_S, t - waitFrom, 2 * DT);
    for (Step s : new Step[] {Step.LOWER_ARM, Step.INTAKE_FLOWER, Step.BACK_FROM_FLOWER, Step.RAISE_ARM,
        Step.SHOOT_GARDEN_SIDE, Step.DRIVE_TO_PARK, Step.PARKED}) {
      runTo(s, CornerSequence.PARK_BY_S + 1);
    }
  }

  @Test
  public void leavesTheCornerByLeaveByEvenIfItGotBackLate() {
    double wait = CornerSequence.WAIT_S;
    CornerSequence.WAIT_S = 20.0;
    try {
      runTo(Step.WAIT_IN_CORNER, 10);
      assertTrue(t + CornerSequence.WAIT_S > CornerSequence.LEAVE_BY_S);
      runTo(Step.DRIVE_TO_FLOWER, 30);
      assertEquals(CornerSequence.LEAVE_BY_S, t, DT);
    } finally {
      CornerSequence.WAIT_S = wait;
    }
  }

  @Test
  public void backsOutOfTheFlowerBeforeParkByThenParks() {
    runTo(Step.INTAKE_FLOWER, 20);
    while (t < CornerSequence.PARK_BY_S - AutoSequence.FLOWER_EXIT_S - 0.1) { // Still driving in, never full
      t += DT;
      seq.update(t, false, false, false, false);
      if (seq.step() != Step.INTAKE_FLOWER) {
        break;
      }
    }
    // The Flower's own limit ends it first here; either way it backs out with the arm down
    assertEquals(Step.BACK_FROM_FLOWER, seq.step());
    assertTrue(seq.armDown());
    runTo(Step.PARKED, 30);
  }

  @Test
  public void parkByEndsTheWaitAndTheVolley() {
    runTo(Step.WAIT_IN_CORNER, 7);
    while (t < CornerSequence.PARK_BY_S + DT) {
      t += DT;
      seq.update(t, false, false, false, false); // Stuck on the drive to the Flower
    }
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
  }

  @Test
  public void shootsOnlyStoppedAtASpotAndParked() {
    assertFalse(seq.shoot(0.1));
    assertFalse("intake off on the way out", seq.intake());
    t = 0.5;
    seq.update(t, true, false, false, false);
    assertTrue(seq.shoot(t));
    assertTrue(seq.intake());
    runTo(Step.DRIVE_TO_FLOWER, 20);
    assertFalse(seq.intake());
    runTo(Step.SHOOT_GARDEN_SIDE, 30);
    assertFalse("not on the drive", seq.shoot(t));
    assertTrue(seq.intake());
    runTo(Step.DRIVE_TO_PARK, 30);
    assertFalse(seq.intake());
    runTo(Step.PARKED, 30);
    t += DT;
    seq.update(t, true, false, false, false);
    assertTrue("the leftovers, parked", seq.shoot(t));
    assertTrue(seq.intake());
    assertFalse(seq.armDown());
  }

  @Test
  public void preloadsIntoScoringThenAudienceForBothAlliances() {
    for (Alliance a : Alliance.values()) {
      assertEquals(Cell.SCORING, CornerSequence.nearCell(a));
      assertEquals(Cell.AUDIENCE, CornerSequence.gardenCell(a));
    }
  }

  @Test
  public void bothVolleysAreFromTheTunedSpotInFrontOfTheirCellFarEnoughFromTheWallsToTurn() {
    for (Alliance a : Alliance.values()) {
      double[] spot = CornerSequence.preloadSpot(a);
      double[] tuned = AutoField.shotSpot(a, Cell.SCORING);
      assertEquals(tuned[0], spot[0], 1e-12);
      assertEquals(tuned[1], spot[1], 1e-12);
      assertEquals("faces our wall on the drive out", AutoField.forAlliance(CornerSequence.CORNER, a)[2], spot[2], 1e-12);
      double[] red = AutoField.forAlliance(AutoField.shotSpot(a, Cell.AUDIENCE), a);
      assertEquals(-AutoField.VOLLEY_SPOT[0], Math.abs(red[0]), 1e-9);
      double halfDiagonal = AutoField.ROBOT_HALF_M * Math.sqrt(2); // Any turn
      assertTrue(AutoField.FIELD_HALF_M - Math.abs(spot[0]) > halfDiagonal);
      assertTrue(AutoField.FIELD_HALF_M - Math.abs(spot[1]) > halfDiagonal);
    }
  }

  @Test
  public void startsOutsideAndParksAtTheGardenEndOfTheLoadingZoneOffOurWall() {
    double[] zone = AutoField.LOADING_ZONE;
    double[] c = CornerSequence.CORNER;
    assertTrue("start clear of the zone", c[0] + AutoField.ROBOT_HALF_M < zone[0]);
    double[] p = CornerSequence.CORNER_PARK;
    assertTrue("centre in the zone", p[0] > zone[0] && p[0] < zone[1]);
    assertTrue("over its Garden-side line by under 10 cm", p[0] + AutoField.ROBOT_HALF_M - zone[1] < 0.10);
    assertTrue(p[1] > zone[2] && p[1] < zone[3]);
    assertEquals(AutoField.PARK_WALL_GAP_M, p[1] - AutoField.ROBOT_HALF_M + AutoField.FIELD_HALF_M, 1e-9);
  }

  @Test
  public void jigglesTheIndexerFromTheFlowerUntilStoppedForTheVolley() {
    runTo(Step.WAIT_IN_CORNER, 7);
    for (double dt = 0; dt < 1; dt += DT) {
      assertFalse("still in the corner", seq.indexerPaused(t + dt));
    }
    runTo(Step.LOWER_ARM, 20);
    double from = t;
    boolean reversed = false;
    for (double dt = 0; dt < 1; dt += DT) {
      reversed |= seq.indexerPaused(from + dt);
    }
    assertTrue(reversed);
    assertFalse("loads first", seq.indexerPaused(from + 0.1));
    assertTrue("then backs off", seq.indexerPaused(from + CornerSequence.JIGGLE_RUN_S + 0.05));
    assertFalse(seq.indexerPaused(from + CornerSequence.JIGGLE_RUN_S + CornerSequence.JIGGLE_REVERSE_S + 0.05));
    runTo(Step.SHOOT_GARDEN_SIDE, 30);
    double s = t + CornerSequence.JIGGLE_RUN_S + 0.05;
    assertTrue("on the drive to the spot", seq.indexerPaused(s));
    t += DT;
    seq.update(t, true, false, false, false); // Arrived: the volley's retry takes over
    assertFalse(seq.indexerPaused(s));
  }

  @Test
  public void parksBackToScoringWithItsTurnedCornerOffOurWallAndInTheZone() {
    double[] zone = AutoField.LOADING_ZONE;
    for (Alliance a : Alliance.values()) {
      assertEquals(Cell.SCORING, CornerSequence.parkCell(a));
      double[] red = AutoField.forAlliance(CornerSequence.parkSpot(a), a);
      assertEquals(CornerSequence.CORNER_PARK[0], red[0], 1e-9);
      double reach = AutoField.ROBOT_HALF_M * (Math.abs(Math.sin(red[2])) + Math.abs(Math.cos(red[2])));
      assertEquals(AutoField.PARK_WALL_GAP_M, red[1] - reach + AutoField.FIELD_HALF_M, 1e-6);
      assertTrue("centre in the zone", red[1] > zone[2] && red[1] < zone[3]);
      double[] spot = CornerSequence.parkSpot(a);
      double[] aim = AutoField.autoAimPoint(a, Cell.SCORING);
      double back = Math.atan2(spot[1] - aim[1], spot[0] - aim[0]);
      assertEquals("shooter to SCORING", 0, Angles.normalize(spot[2] - back), 1e-3);
    }
  }

  private static boolean inFlowerKeepOut(double x, double y) {
    double[] k = AutoField.WALL_FLOWER_KEEP_OUT;
    return x > k[0] && x < k[1] && y < k[2];
  }

  @Test
  public void theDrivesKeepClearOfOurWallsFlower() {
    double[] a = CornerSequence.CORNER_WAIT;
    double[] e = CornerSequence.EDGE_END;
    double[] b = CornerSequence.FLOWER_BEND;
    double[] f = AutoField.WALL_FLOWER;
    double[] shot = AutoField.shotSpot(Alliance.RED, Cell.AUDIENCE);
    double[] park = CornerSequence.parkSpot(Alliance.RED);
    for (int i = 0; i <= 100; i++) {
      double s = i / 100.0;
      assertFalse("along the wall", inFlowerKeepOut(a[0] + s * (e[0] - a[0]), a[1] + s * (e[1] - a[1])));
      double x = (1 - s) * (1 - s) * e[0] + 2 * s * (1 - s) * b[0] + s * s * f[0];
      double y = (1 - s) * (1 - s) * e[1] + 2 * s * (1 - s) * b[1] + s * s * f[1];
      assertFalse("the bend at " + s, inFlowerKeepOut(x, y));
      assertFalse("to park", inFlowerKeepOut(shot[0] + s * (park[0] - shot[0]), shot[1] + s * (park[1] - shot[1])));
    }
    assertEquals("along the wall, just off it", -(AutoField.AT_WALL_M - CornerSequence.EDGE_GAP_M), e[1], 1e-9);
  }
}
