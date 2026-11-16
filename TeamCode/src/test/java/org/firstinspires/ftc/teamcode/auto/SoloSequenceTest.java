package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.SoloSequence.Step;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.junit.Test;

public class SoloSequenceTest {
  private static final double DT = 0.02;
  private final SoloSequence seq = new SoloSequence(0);
  private final List<Step> seen = new ArrayList<>();
  private double t = 0;

  /** One loop; the step's drive is done once {@code arrived}. Notes each new step. */
  private void loop(boolean arrived, boolean ballLeft, boolean feederEmpty, boolean stackFull) {
    t += DT;
    seq.update(t, arrived, ballLeft, feederEmpty, stackFull);
    if (seq.takeNewStep() && (seen.isEmpty() || seen.get(seen.size() - 1) != seq.step())) {
      seen.add(seq.step());
    }
  }

  /** Every drive arrives at once, the feeder is empty and a Flower full: the quickest way through. */
  private void quickTo(Step target) {
    double from = t;
    while (seq.step() != target) {
      loop(true, false, true, true);
      assertTrue("stuck in " + seq.step() + " before " + target, t - from < 30);
    }
  }

  private void runFor(double seconds, boolean arrived) {
    double end = t + seconds;
    while (t < end - 1e-9) {
      loop(arrived, false, false, false);
    }
  }

  @Test
  public void opensWithThePreloadsTheGardenAndTheCheck() {
    seen.add(seq.step());
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    assertEquals(Arrays.asList(Step.SHOOT_PRELOAD, Step.FACE_GARDEN, Step.ONTO_GARDEN_WALL, Step.SWEEP_GARDEN,
        Step.CHECK_TIP), seen);
    assertNull("not decided before the check", seq.tipped());
  }

  @Test
  public void notTippedShootsTheGardenNearThenWallFlowerParkRearFlowerAndParks() {
    seq.tipCheck(null, true); // The prediction says tipped, the tags say not: the tags win
    quickTo(Step.CHECK_TIP);
    seen.clear();
    seq.tipCheck(false, true);
    quickTo(Step.PARKED);
    assertEquals(Boolean.FALSE, seq.tipped());
    assertTrue(seq.tipFromTags());
    assertEquals(Arrays.asList(Step.SHOOT_NEAR,
        Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER,
        Step.RAISE_ARM_WALL, Step.SHOOT_PARK,
        Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.RAISE_ARM_REAR, Step.DRIVE_TO_PARK, Step.PARKED), seen);
  }

  @Test
  public void tippedShootsTheGardenFromTheParkThenRearFlowerFarWallFlowerNearAndParks() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    seen.clear();
    seq.tipCheck(true, false);
    quickTo(Step.PARKED);
    assertEquals(Boolean.TRUE, seq.tipped());
    assertEquals(Arrays.asList(Step.SHOOT_PARK,
        Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.RAISE_ARM_REAR, Step.SHOOT_FAR,
        Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER,
        Step.RAISE_ARM_WALL, Step.SHOOT_NEAR, Step.DRIVE_TO_PARK, Step.PARKED), seen);
  }

  @Test
  public void theTagsDecideMidDriveWithoutStoppingAtTheSpot() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    runFor(0.3, false); // Still driving out of the corner, no fresh frames
    assertEquals(Step.CHECK_TIP, seq.step());
    seq.tipCheck(true, false);
    loop(false, false, false, false);
    assertEquals("on to the park at once", Step.SHOOT_PARK, seq.step());
  }

  @Test
  public void withNoTagsThePredictionDecidesShortlyAfterTheSpot() {
    seq.tipCheck(null, true);
    quickTo(Step.CHECK_TIP);
    runFor(1.0, false);
    assertEquals("waits while driving", Step.CHECK_TIP, seq.step());
    runFor(SoloSequence.TIP_CHECK_WAIT_S - 0.05, true);
    assertEquals("waits a moment at the spot", Step.CHECK_TIP, seq.step());
    runFor(0.1, true);
    assertEquals(Step.SHOOT_PARK, seq.step());
    assertFalse(seq.tipFromTags());
  }

  @Test
  public void aDriveThatNeverArrivesStillDecidesByTheLongest() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    runFor(SoloSequence.TIP_CHECK_MAX_S + 0.05, false);
    assertEquals(Step.SHOOT_NEAR, seq.step());
  }

  @Test
  public void noFlowerIsStartedAfterTheLatestAndItParksInstead() {
    seq.tipCheck(false, false);
    quickTo(Step.SHOOT_NEAR);
    while (t < SoloSequence.LATEST_FLOWER_S) {
      loop(false, false, false, false); // Held off the spot until late
    }
    loop(true, false, true, false);
    while (seq.step() == Step.SHOOT_NEAR) {
      loop(true, false, true, false);
    }
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
  }

  @Test
  public void leavesAFlowerInTimeToPark() {
    seq.tipCheck(false, false);
    quickTo(Step.DRIVE_TO_WALL_FLOWER); // Started in time, then slow to get there
    double exitS = SoloSequence.PARK_BY_S - AutoSequence.FLOWER_EXIT_S;
    while (t < exitS - AutoSequence.ARM_MOVE_S - 1.0) {
      loop(false, false, false, false);
    }
    quickTo(Step.INTAKE_WALL_FLOWER);
    while (seq.step() == Step.INTAKE_WALL_FLOWER) {
      loop(false, false, false, false); // Short of the Flower, never full
    }
    assertEquals(Step.BACK_FROM_WALL_FLOWER, seq.step());
    assertEquals("backs out at the exit time", exitS, t, DT * 1.5);
    quickTo(Step.DRIVE_TO_PARK);
    assertTrue("parking by " + t, t < SoloSequence.PARK_BY_S + 1.0);
  }

  @Test
  public void atTheParkByTimeAVolleyFromTheParkJustStaysParked() {
    seq.tipCheck(true, false);
    quickTo(Step.SHOOT_PARK);
    while (t < SoloSequence.PARK_BY_S - DT) {
      seq.update(t += DT, false, false, false, false);
    }
    loop(false, false, false, false);
    assertEquals(Step.PARKED, seq.step());
  }

  @Test
  public void aDriveToParkAlreadyUnderWayIsNotRestartedAtTheParkByTime() {
    seq.tipCheck(false, false);
    quickTo(Step.DRIVE_TO_PARK);
    seq.takeNewStep();
    while (t < SoloSequence.PARK_BY_S + 0.5) {
      loop(false, false, false, false);
    }
    assertEquals(Step.DRIVE_TO_PARK, seq.step());
    assertFalse("no new drive", seq.takeNewStep());
  }

  @Test
  public void eachVolleyAimsAtTheCellOfItsSpot() {
    assertEquals(Step.SHOOT_PRELOAD, seq.nextVolley());
    seq.tipCheck(null, false);
    quickTo(Step.FACE_GARDEN);
    assertEquals("likely not tipped", Step.SHOOT_NEAR, seq.nextVolley());
    seq.tipCheck(null, true);
    quickTo(Step.CHECK_TIP);
    assertEquals("likely tipped", Step.SHOOT_PARK, seq.nextVolley());
    seq.tipCheck(false, true);
    assertEquals("the tags say not", Step.SHOOT_NEAR, seq.nextVolley());
    quickTo(Step.DRIVE_TO_WALL_FLOWER);
    assertEquals(Step.SHOOT_PARK, seq.nextVolley());
  }

  @Test
  public void shootsOnlyStoppedAndIntakesThroughPickupsAndVolleys() {
    seq.tipCheck(false, false);
    assertFalse(seq.shoot(t));
    assertFalse("preloads: not into the spinning wheels on the way", seq.intake());
    loop(true, false, false, false);
    assertTrue(seq.shoot(t));
    assertTrue(seq.intake());
    quickTo(Step.CHECK_TIP);
    assertTrue("the Garden's balls up from the intake's mouth", seq.intake());
    quickTo(Step.DRIVE_TO_WALL_FLOWER);
    assertFalse(seq.intake());
    assertFalse(seq.armDown());
    quickTo(Step.INTAKE_WALL_FLOWER);
    assertTrue(seq.armDown());
  }

  @Test
  public void theNearCellIsTheOneUpAtTheStartBesideOurGarden() {
    assertEquals(Cell.AUDIENCE, SoloSequence.nearCell(Alliance.RED));
    assertEquals(Cell.SCORING, SoloSequence.nearCell(Alliance.BLUE));
    for (Alliance a : Alliance.values()) {
      assertEquals(new VisionCorrector().upCell(a), SoloSequence.nearCell(a));
      double[] start = AutoField.forAlliance(SoloSequence.START, a);
      double[] near = AutoField.autoAimPoint(a, SoloSequence.nearCell(a));
      double[] far = AutoField.autoAimPoint(a, SoloSequence.parkCell(a));
      assertTrue(a + ": near Cell nearer the start",
          Math.hypot(near[0] - start[0], near[1] - start[1]) < Math.hypot(far[0] - start[0], far[1] - start[1]));
      double[] park = SoloSequence.parkSpot(a);
      assertTrue(a + ": park Cell nearer the park",
          Math.hypot(far[0] - park[0], far[1] - park[1]) < Math.hypot(near[0] - park[0], near[1] - park[1]));
    }
  }

  @Test
  public void startsTouchingTheWallBesideTheGardenOutOfTheLoadingZone() {
    double[] s = SoloSequence.START;
    assertEquals("touching the audience wall", AutoField.FIELD_HALF_M, s[0] + AutoField.ROBOT_HALF_M, 1e-9);
    assertEquals("side 2 cm past the Garden's tile", -1.22 + 0.02, s[1] - AutoField.ROBOT_HALF_M, 0.005);
    assertTrue("out of the loading zone", s[0] - AutoField.ROBOT_HALF_M > AutoField.LOADING_ZONE[1]);
    double[] p = SoloSequence.PRELOAD_SPOT;
    double halfDiagonal = AutoField.ROBOT_HALF_M * Math.sqrt(2);
    assertTrue("room to turn off the wall", p[0] + halfDiagonal < AutoField.FIELD_HALF_M);
  }
}
