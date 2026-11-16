package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.SoloSequence.Step;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
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
        Step.SHOOT_PARK,
        Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.DRIVE_TO_PARK, Step.PARKED), seen);
  }

  @Test
  public void tippedShootsTheGardenFromTheParkThenRearFlowerFarWallFlowerNearAndParks() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    seen.clear();
    seq.tipCheck(true, false);
    quickTo(Step.SHOOT_PARK);
    seq.tipCheck(false, true); // At our wall's Flower the tags show the near Cell up again: the far Cell tipped back
    quickTo(Step.PARKED);
    assertEquals(Boolean.TRUE, seq.tipped());
    assertEquals(Boolean.TRUE, seq.farTipped());
    assertTrue(seq.farTipFromTags());
    assertEquals(Arrays.asList(Step.SHOOT_PARK,
        Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.SHOOT_FAR,
        Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER,
        Step.SHOOT_NEAR, Step.DRIVE_TO_PARK, Step.PARKED), seen);
  }

  @Test
  public void tippedWithTheFarCellStillUpShootsItAgainFromTheParkAndStays() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    seen.clear();
    seq.tipCheck(true, false); // The near Cell stays down: the far Cell has not tipped back by the recheck
    quickTo(Step.PARKED);
    assertEquals(Boolean.FALSE, seq.farTipped());
    assertTrue(seq.farTipFromTags());
    assertEquals(Arrays.asList(Step.SHOOT_PARK,
        Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR, Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER,
        Step.SHOOT_FAR,
        Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER,
        Step.SHOOT_PARK, Step.PARKED), seen);
  }

  @Test
  public void withNoTagsAtTheWallFlowerTheBallCountDecidesTheRecheck() {
    for (boolean nearDown : new boolean[] {false, true}) {
      SoloSequence s = new SoloSequence(0);
      double now = 0;
      s.tipCheck(true, false);
      while (s.step() != Step.BACK_FROM_WALL_FLOWER) {
        s.update(now += DT, true, false, true, true);
      }
      s.tipCheck(null, nearDown);
      while (s.step() == Step.BACK_FROM_WALL_FLOWER) {
        s.update(now += DT, true, false, true, true);
      }
      assertEquals(!nearDown, s.farTipped());
      assertFalse(s.farTipFromTags());
      assertEquals(nearDown ? Step.SHOOT_PARK : Step.SHOOT_NEAR, s.step());
    }
  }

  @Test
  public void theRecheckWatchesTheHiveOnlyAtOurWallsFlowerOnTheTippedRoute() {
    seq.tipCheck(null, false);
    quickTo(Step.CHECK_TIP);
    assertTrue("the check", seq.watchingHive());
    seq.tipCheck(true, false);
    quickTo(Step.SHOOT_FAR);
    assertFalse("the far Cell's volley still landing", seq.watchingHive());
    quickTo(Step.DRIVE_TO_WALL_FLOWER);
    assertFalse("turning on the way", seq.watchingHive());
    for (Step s : Arrays.asList(Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER)) {
      quickTo(s);
      assertTrue(s + ": camera to the Hive", seq.watchingHive());
    }
    seq.tipCheck(false, false);
    assertEquals("the near Cell up again", Step.SHOOT_NEAR, seq.nextVolley());
    seq.tipCheck(null, true);
    assertEquals("no tags: the ball count says still up", Step.SHOOT_PARK, seq.nextVolley());
    seq.tipCheck(false, true);
    quickTo(Step.SHOOT_NEAR);
    assertFalse("rechecked", seq.watchingHive());

    SoloSequence notTipped = new SoloSequence(0);
    double now = 0;
    notTipped.tipCheck(false, false);
    while (notTipped.step() != Step.INTAKE_WALL_FLOWER) {
      notTipped.update(now += DT, true, false, true, false);
      assertTrue(notTipped.step() == Step.CHECK_TIP || !notTipped.watchingHive());
    }
  }
  @Test
  public void theArmGoesDownOnTheApproachAndUpOnTheNextDriveWithNoStopForIt() {
    seq.tipCheck(false, false);
    quickTo(Step.DRIVE_TO_WALL_FLOWER);
    assertFalse(seq.armDown());
    loop(true, false, false, false); // Near enough the stop: it closes the rest as the arm goes down
    assertEquals(Step.LOWER_ARM_WALL, seq.step());
    assertTrue("going down short of the stop", seq.armDown());
    double lowering = t;
    while (seq.step() == Step.LOWER_ARM_WALL) {
      loop(true, false, false, false); // At the stop at once
    }
    assertEquals(Step.INTAKE_WALL_FLOWER, seq.step());
    assertEquals("in once the arm has had its move", lowering + AutoSequence.ARM_MOVE_S, t, DT * 1.5);
    quickTo(Step.BACK_FROM_WALL_FLOWER);
    assertTrue("down backing out", seq.armDown());
    loop(true, false, false, false);
    assertEquals("on from backing out at once", Step.SHOOT_PARK, seq.step());
    assertFalse("coming up on the drive", seq.armDown());
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
    double[] p = SoloSequence.preloadSpot(Alliance.RED);
    double halfDiagonal = AutoField.ROBOT_HALF_M * Math.sqrt(2);
    assertTrue("room to turn off the wall", p[0] + halfDiagonal < AutoField.FIELD_HALF_M);
  }

  @Test
  public void longDrivesGoOnOneOfTheRobotsAxesNeverDiagonally() {
    for (double from = -3.1; from < Math.PI; from += 0.3) {
      for (double to = -3.1; to < Math.PI; to += 0.3) {
        for (double travel = -3.1; travel < Math.PI; travel += 0.4) {
          double off = Angles.normalize(SoloSequence.alongAxis(from, to, travel, 2.0) - travel);
          assertEquals(0, off - Math.round(off / (Math.PI / 2)) * (Math.PI / 2), 1e-9);
        }
      }
    }
    assertEquals("facing the way it goes at both ends: straight ahead", 0.3,
        SoloSequence.alongAxis(0.3, 0.3, 0.3, 2.0), 1e-9);
    double sideOn = 0.3 + Math.PI / 2;
    assertEquals("side-on at both ends: a strafe, no turning", sideOn,
        SoloSequence.alongAxis(sideOn, sideOn, 0.3, 2.0), 1e-9);
  }

  @Test
  public void theDriveToTheParkBendsOnlyWhereStraightWouldCrossOurWallsFlower() {
    double[] park = SoloSequence.parkSpot(Alliance.RED);
    assertNull("from the preload spot: straight", SoloSequence.parkControl(SoloSequence.preloadSpot(Alliance.RED)));
    double[] corner = {1.43, -1.43, 0}; // The Garden's corner, where the tip check decides
    assertTrue(SoloSequence.crossesWallFlower(corner, park));
    double[] c = SoloSequence.parkControl(corner);
    assertSame(SoloSequence.PARK_ENTRY_CONTROL, c);
    double[] keepOut = AutoField.WALL_FLOWER_KEEP_OUT;
    for (int i = 0; i <= 200; i++) {
      double s = i / 200.0;
      double u = 1 - s;
      double x = u * u * corner[0] + 2 * u * s * c[0] + s * s * park[0];
      double y = u * u * corner[1] + 2 * u * s * c[1] + s * s * park[1];
      if (x > keepOut[0] && x < keepOut[1]) {
        assertTrue("bent drive at " + x + ", " + y, y > keepOut[2] + 0.05);
      }
    }
  }
}
