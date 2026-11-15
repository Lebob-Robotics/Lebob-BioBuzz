package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.firstinspires.ftc.teamcode.utils.VisionResult.Reason;
import org.firstinspires.ftc.teamcode.utils.VisionResult.TagReject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Ported from Sean's vision tests on feature/sotm, using this branch's camera mounts. */
public class VisionCorrectorTest {
  private static final double TOLERANCE = 1e-9;
  private static final double MM_TOLERANCE = 1e-3;
  private static final long T0 = 1_000_000_000L;
  private static final long MS = 1_000_000L;

  private static final CameraMount LEFT = ShootOnTheMoveConstants.LEFT_CAMERA;
  private static final CameraMount RIGHT = ShootOnTheMoveConstants.RIGHT_CAMERA;

  // Robot used for most tests: between the Hives
  private static final Pose TRUE_POSE = new Pose(0.1, -0.9, -Math.PI / 2);

  private double[] savedConstants; // The constants are live-tunable statics, so tests put them back
  private int savedConfirmFrames;
  private boolean savedVisionHeading;
  private VisionCorrector corrector;

  @Before
  public void setUp() {
    savedConfirmFrames = ShootOnTheMoveConstants.TIP_CONFIRM_FRAMES;
    savedVisionHeading = ShootOnTheMoveConstants.VISION_HEADING;
    savedConstants = new double[] {
        ShootOnTheMoveConstants.MIN_DECISION_MARGIN, ShootOnTheMoveConstants.MAX_TAG_RANGE_M,
        ShootOnTheMoveConstants.MAX_SPEED_FOR_VISION_MPS, ShootOnTheMoveConstants.MAX_OMEGA_FOR_VISION_RAD_S,
        ShootOnTheMoveConstants.MAX_FRAME_AGE_S, ShootOnTheMoveConstants.MAX_SPREAD_M,
        ShootOnTheMoveConstants.MAX_JUMP_M, ShootOnTheMoveConstants.CORRECTION_GAIN,
        ShootOnTheMoveConstants.Z_TOL_M, ShootOnTheMoveConstants.FIELD_HALF_SIZE_M,
        ShootOnTheMoveConstants.ROBOT_HALF_SIZE_M};
    corrector = new VisionCorrector();
  }

  @After
  public void restoreConstants() {
    ShootOnTheMoveConstants.TIP_CONFIRM_FRAMES = savedConfirmFrames;
    ShootOnTheMoveConstants.VISION_HEADING = savedVisionHeading;
    ShootOnTheMoveConstants.MIN_DECISION_MARGIN = savedConstants[0];
    ShootOnTheMoveConstants.MAX_TAG_RANGE_M = savedConstants[1];
    ShootOnTheMoveConstants.MAX_SPEED_FOR_VISION_MPS = savedConstants[2];
    ShootOnTheMoveConstants.MAX_OMEGA_FOR_VISION_RAD_S = savedConstants[3];
    ShootOnTheMoveConstants.MAX_FRAME_AGE_S = savedConstants[4];
    ShootOnTheMoveConstants.MAX_SPREAD_M = savedConstants[5];
    ShootOnTheMoveConstants.MAX_JUMP_M = savedConstants[6];
    ShootOnTheMoveConstants.CORRECTION_GAIN = savedConstants[7];
    ShootOnTheMoveConstants.Z_TOL_M = savedConstants[8];
    ShootOnTheMoveConstants.FIELD_HALF_SIZE_M = savedConstants[9];
    ShootOnTheMoveConstants.ROBOT_HALF_SIZE_M = savedConstants[10];
  }

  // ---- Helpers ------------------------------------------------------------

  /**
   * What a camera would report for a tag, worked out from first principles: the tag's offset from the
   * camera, projected on the camera's right, forward and up axes (up built as right cross forward).
   */
  private static TagObservation observe(CameraMount camera, Pose robot, int id, Cell up,
      double tagHeightOverrideM, long frameNanos) {
    double[] tag = VisionCorrector.tagFieldPosition(id, up);
    double tagHeightM = Double.isNaN(tagHeightOverrideM) ? tag[2] : tagHeightOverrideM;
    double dx = tag[0] - robot.xM;
    double dy = tag[1] - robot.yM;
    double cos = Math.cos(robot.headingRad);
    double sin = Math.sin(robot.headingRad);
    double[] fromCamera = {
        cos * dx + sin * dy - camera.forwardM,
        -sin * dx + cos * dy - camera.leftM,
        tagHeightM - camera.heightM};
    double[] forward = {
        Math.cos(camera.pitchRad) * Math.cos(camera.yawRad),
        Math.cos(camera.pitchRad) * Math.sin(camera.yawRad),
        Math.sin(camera.pitchRad)};
    double[] right = {Math.sin(camera.yawRad), -Math.cos(camera.yawRad), 0};
    double[] upAxis = {
        right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0]};
    // Rotate the physical right/up basis about the lens axis for rolled camera mounts.
    double[] cameraRight = new double[3];
    double[] cameraUp = new double[3];
    for (int axis = 0; axis < 3; axis++) {
      cameraRight[axis] = Math.cos(camera.rollRad) * right[axis] + Math.sin(camera.rollRad) * upAxis[axis];
      cameraUp[axis] = -Math.sin(camera.rollRad) * right[axis] + Math.cos(camera.rollRad) * upAxis[axis];
    }
    return new TagObservation(id, camera, dot(fromCamera, cameraRight), dot(fromCamera, forward),
        dot(fromCamera, cameraUp), 100, frameNanos);
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  /** Seen by the left camera at T0 + 50 ms. */
  private static TagObservation observe(int id, Cell up, Pose robot) {
    return observe(LEFT, robot, id, up, Double.NaN, T0 + 50 * MS);
  }

  /** Odometry sat still at the given pose from T0 to T0 + 100 ms. */
  private static PoseHistory stillHistory(Pose odometry) {
    PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
    history.add(T0, odometry);
    history.add(T0 + 100 * MS, odometry);
    return history;
  }

  private static Pose offsetBy(Pose pose, double dx, double dy) {
    return new Pose(pose.xM + dx, pose.yM + dy, pose.headingRad);
  }

  private VisionResult run(List<TagObservation> observations, Pose odometry, boolean poseValid) {
    return corrector.process(observations, stillHistory(odometry), odometry, 0, 0, 0, poseValid, T0 + 100 * MS);
  }

  private static List<TagObservation> list(TagObservation... observations) {
    return new ArrayList<>(Arrays.asList(observations));
  }

  // ---- Field and camera maths ---------------------------------------------

  @Test public void upCellAimPointIsOutsideTheTagLineCentre() {
    double outM = ShootOnTheMoveConstants.CLUSTER_UP_X_M + ShootOnTheMoveConstants.CELL_REF_UP_TOWARD_IN * 0.0254;
    double[] redScoring = VisionCorrector.upCellAimPoint(Alliance.RED, Cell.SCORING);
    assertEquals(-outM, redScoring[0], TOLERANCE);
    assertEquals(-12.75 * 0.0254, redScoring[1], TOLERANCE);
    double[] blueAudience = VisionCorrector.upCellAimPoint(Alliance.BLUE, Cell.AUDIENCE);
    assertEquals(outM, blueAudience[0], TOLERANCE);
    assertEquals(12.75 * 0.0254, blueAudience[1], TOLERANCE);
    // Where the sightings put RED SCORING up on 5 Nov (081558): about (-0.46, -0.29)
    assertEquals(-0.46, redScoring[0], 0.03);
    assertEquals(-0.29, redScoring[1], 0.04);
  }

  @Test public void upCellIsNotConfirmedBeforeAnyTags() {
    VisionCorrector corrector = new VisionCorrector();
    assertFalse(corrector.upCellConfirmed(Alliance.RED));
    assertFalse(corrector.upCellConfirmed(Alliance.BLUE));
  }

  @Test
  public void tagFieldPositionsFollowTheTipState() {
    double[] downCluster = VisionCorrector.tagFieldPosition(30, Cell.AUDIENCE); // RED SCORING, AUDIENCE up
    assertEquals(-0.2949, downCluster[0], TOLERANCE);
    assertEquals(-6.25 * 0.0254, downCluster[1], TOLERANCE);
    assertEquals(0.9022, downCluster[2], TOLERANCE);
    double[] upCluster = VisionCorrector.tagFieldPosition(30, Cell.SCORING);
    assertEquals(-0.3329, upCluster[0], TOLERANCE);
    assertEquals(1.2646, upCluster[2], TOLERANCE);
    double[] redAudienceUp = VisionCorrector.tagFieldPosition(34, Cell.AUDIENCE);
    assertEquals(0.3329, redAudienceUp[0], TOLERANCE);
    assertEquals(-19.25 * 0.0254, redAudienceUp[1], TOLERANCE);
    double[] blueAudienceDown = VisionCorrector.tagFieldPosition(41, Cell.SCORING);
    assertEquals(0.2949, blueAudienceDown[0], TOLERANCE);
    assertEquals(19.25 * 0.0254, blueAudienceDown[1], TOLERANCE);
    assertEquals(6.25 * 0.0254, VisionCorrector.tagFieldPosition(45, Cell.SCORING)[1], TOLERANCE);
    assertNull(VisionCorrector.tagFieldPosition(29, Cell.AUDIENCE));
    assertNull(VisionCorrector.tagFieldPosition(46, Cell.AUDIENCE));
    assertNull(VisionCorrector.tagFieldPosition(-1, Cell.AUDIENCE));

    assertEquals(Alliance.RED, VisionCorrector.tagAlliance(37));
    assertEquals(Alliance.BLUE, VisionCorrector.tagAlliance(38));
    assertNull(VisionCorrector.tagAlliance(20));
    assertEquals(Cell.SCORING, VisionCorrector.tagCell(33));
    assertEquals(Cell.AUDIENCE, VisionCorrector.tagCell(34));
    assertEquals(Cell.AUDIENCE, VisionCorrector.tagCell(41));
    assertEquals(Cell.SCORING, VisionCorrector.tagCell(42));
    assertNull(VisionCorrector.tagCell(0));
  }

  @Test
  public void cameraAxesPointTheRightWay() {
    // The left camera looks backward and to the left, upside down, so its picture-right points backward and right
    double[] ahead = LEFT.toRobotFrame(0, 1, 0);
    assertTrue(ahead[0] < LEFT.forwardM && ahead[1] > LEFT.leftM && ahead[2] > LEFT.heightM);
    double[] right = LEFT.toRobotFrame(1, 0, 0);
    assertTrue(right[0] < LEFT.forwardM && right[1] < LEFT.leftM);
    assertEquals(LEFT.heightM, right[2], TOLERANCE);
    double[] aheadRight = RIGHT.toRobotFrame(0, 1, 0); // The right camera is the mirror image
    assertTrue(aheadRight[0] < RIGHT.forwardM && aheadRight[1] < RIGHT.leftM && aheadRight[2] > RIGHT.heightM);
    double[] up = LEFT.toRobotFrame(0, 0, 1); // Upside down, picture-up lowers the point by cos(pitch) per metre
    assertEquals(LEFT.heightM - Math.cos(LEFT.pitchRad), up[2], TOLERANCE);
  }

  @Test
  public void measuredHeightDoesNotDependOnHeadingOrPosition() {
    for (double heading : new double[] {0, 1, -2}) {
      TagObservation tag = observe(RIGHT, new Pose(-0.3, 0.2, heading), 36, Cell.AUDIENCE, Double.NaN, 0);
      double[] inRobot = tag.camera.toRobotFrame(tag.rightM, tag.forwardM, tag.upM);
      assertEquals(1.2646, inRobot[2], TOLERANCE);
    }
  }

  @Test
  public void aPerfectViewRecoversThePoseAtEveryHeadingAndTipState() {
    double[] headings = {-Math.PI / 2, 0.0, 2.0, -2.7};
    for (double heading : headings) {
      for (Cell redUp : Cell.values()) {
        corrector = new VisionCorrector();
        Pose truth = new Pose(0.1, -0.9, heading);
        Pose odometry = offsetBy(truth, 0.04, -0.03); // Odometry is wrong by (+0.04, -0.03)
        // Each tag is placed using the tip state its own height shows, so the corrector's belief does not matter
        List<TagObservation> observations = new ArrayList<>();
        for (CameraMount camera : new CameraMount[] {LEFT, RIGHT}) {
          for (int id = 30; id <= 45; id++) {
            Cell up = VisionCorrector.tagAlliance(id) == Alliance.RED ? redUp : VisionCorrector.otherCell(redUp);
            observations.add(observe(camera, truth, id, up, Double.NaN, T0 + 50 * MS));
          }
        }
        VisionResult result = run(observations, odometry, true);
        assertTrue("heading " + heading + " " + result.reason + " " + result.firstTagReject, result.accepted);
        assertEquals(32, result.tagsUsed);
        assertEquals(-0.04, result.dxM, MM_TOLERANCE);
        assertEquals(0.03, result.dyM, MM_TOLERANCE);
        assertEquals(0.0, result.spreadM, 1e-6);
      }
    }
  }

  // ---- Tip detection ------------------------------------------------------

  @Test
  public void tipDetectionNeedsThreeFrames() {
    // The red Hive starts with AUDIENCE up, but the tags show SCORING up
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
    for (int frame = 1; frame <= 3; frame++) {
      VisionResult result = run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
      if (frame < 3) {
        assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
        assertFalse(result.redFlipped);
        assertFalse(corrector.upCellConfirmed(Alliance.RED)); // Still the starting guess
      } else {
        assertEquals(Cell.SCORING, corrector.upCell(Alliance.RED));
        assertTrue(result.redFlipped);
        assertTrue(corrector.upCellConfirmed(Alliance.RED));
        assertFalse(result.blueFlipped);
      }
    }
    assertEquals(Cell.SCORING, corrector.upCell(Alliance.BLUE)); // Untouched
  }

  @Test
  public void tagsAgreeingWithTheStartingGuessConfirmItAtOnce() {
    run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
    assertTrue(corrector.upCellConfirmed(Alliance.RED));
    assertFalse(corrector.upCellConfirmed(Alliance.BLUE));
  }

  @Test
  public void tipVotesMustBeConsecutive() {
    TagObservation flip = observe(31, Cell.SCORING, TRUE_POSE);
    TagObservation same = observe(31, Cell.AUDIENCE, TRUE_POSE);
    run(list(flip), TRUE_POSE, true);
    run(list(flip), TRUE_POSE, true);
    run(list(same), TRUE_POSE, true); // Back to the current state: the votes reset
    run(list(flip), TRUE_POSE, true);
    run(list(flip), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
    run(list(flip), TRUE_POSE, true);
    assertEquals(Cell.SCORING, corrector.upCell(Alliance.RED));
  }

  @Test
  public void aTagAtTheDownHeightMeansTheOtherCellIsUp() {
    // Blue starts with SCORING up. A BLUE SCORING tag (42) at the down height means AUDIENCE is up
    ShootOnTheMoveConstants.TIP_CONFIRM_FRAMES = 1;
    VisionResult result = run(list(observe(42, Cell.AUDIENCE, TRUE_POSE)), TRUE_POSE, true);
    assertTrue(result.blueFlipped);
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.BLUE));
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED)); // Untouched
  }

  @Test
  public void aConfirmedTipKeepsRunningWithoutFlippingBack() {
    ShootOnTheMoveConstants.TIP_CONFIRM_FRAMES = 1;
    run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
    assertEquals(Cell.SCORING, corrector.upCell(Alliance.RED));
    VisionResult result = run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
    assertFalse(result.redFlipped);
    assertEquals(Cell.SCORING, corrector.upCell(Alliance.RED));
  }

  @Test
  public void resetClearsPartialTipVotes() {
    TagObservation flip = observe(31, Cell.SCORING, TRUE_POSE);
    run(list(flip), TRUE_POSE, true);
    run(list(flip), TRUE_POSE, true);
    corrector.reset();
    run(list(flip), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
  }

  @Test
  public void aBadHeightCastsNoTipVote() {
    ShootOnTheMoveConstants.TIP_CONFIRM_FRAMES = 1;
    run(list(observe(LEFT, TRUE_POSE, 31, Cell.SCORING, 0.5, T0 + 50 * MS)), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, corrector.upCell(Alliance.RED));
  }

  // ---- Rejecting tags -----------------------------------------------------

  private void assertTagRejected(TagObservation tag, TagReject why) {
    VisionResult result = run(list(tag), TRUE_POSE, true);
    assertFalse(result.accepted);
    assertEquals(Reason.NO_VALID_TAGS, result.reason);
    assertEquals(why, result.firstTagReject);
    assertEquals(1, result.tagsRejected);
    assertEquals(0, result.tagsUsed);
    assertNull(result.newPose);
  }

  @Test
  public void rejectsUnknownAndClusterIdsAndMissingPoses() {
    TagObservation good = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(new TagObservation(20, good.camera, good.rightM, good.forwardM, good.upM, 100,
        good.frameTimeNanos), TagReject.NOT_HIVE_TAG);
    assertTagRejected(new TagObservation(-1, good.camera, good.rightM, good.forwardM, good.upM, Double.NaN,
        good.frameTimeNanos), TagReject.NOT_HIVE_TAG);
    assertTagRejected(new TagObservation(31, good.camera, Double.NaN, Double.NaN, Double.NaN, 100,
        good.frameTimeNanos), TagReject.NO_POSE);
  }

  @Test
  public void rejectsALowDecisionMargin() {
    TagObservation good = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(new TagObservation(31, good.camera, good.rightM, good.forwardM, good.upM, 19.9,
        good.frameTimeNanos), TagReject.LOW_MARGIN);
    assertTrue(run(list(new TagObservation(31, good.camera, good.rightM, good.forwardM, good.upM, 20,
        good.frameTimeNanos)), TRUE_POSE, true).accepted);
  }

  @Test
  public void rejectsATagTooFarAway() {
    TagObservation good = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(new TagObservation(31, good.camera, 0, 3.5, 0, 100, good.frameTimeNanos),
        TagReject.TOO_FAR);
    ShootOnTheMoveConstants.MAX_TAG_RANGE_M = 0.1;
    assertTagRejected(good, TagReject.TOO_FAR);
  }

  @Test
  public void rejectsAnImplausibleHeight() {
    assertTagRejected(observe(LEFT, TRUE_POSE, 31, Cell.AUDIENCE, 0.5, T0 + 50 * MS), TagReject.BAD_Z);
    assertTagRejected(observe(LEFT, TRUE_POSE, 34, Cell.AUDIENCE, 1.2646 + 0.11, T0 + 50 * MS),
        TagReject.BAD_Z); // Just outside the tolerance of the up height
    assertEquals(1, run(list(observe(LEFT, TRUE_POSE, 34, Cell.AUDIENCE, 1.2646 + 0.05, T0 + 50 * MS)),
        TRUE_POSE, true).tagsUsed); // Just inside it
  }

  @Test
  public void rejectsWhenMovingTooFastOrTurningTooFast() {
    List<TagObservation> observations = list(observe(31, Cell.AUDIENCE, TRUE_POSE));
    PoseHistory history = stillHistory(TRUE_POSE);
    double limit = ShootOnTheMoveConstants.MAX_SPEED_FOR_VISION_MPS;
    VisionResult fast = corrector.process(observations, history, TRUE_POSE, limit, limit, 0, true, T0 + 100 * MS);
    assertEquals(Reason.TOO_FAST, fast.reason);
    assertFalse(fast.accepted);
    assertTrue(corrector.process(observations, history, TRUE_POSE, limit, 0, 0, true, T0 + 100 * MS).accepted);
    assertEquals(Reason.TOO_FAST_TURNING,
        corrector.process(observations, history, TRUE_POSE, 0, 0, 1.1, true, T0 + 100 * MS).reason);
    assertEquals(Reason.TOO_FAST_TURNING,
        corrector.process(observations, history, TRUE_POSE, 0, 0, -1.1, true, T0 + 100 * MS).reason);
  }

  @Test
  public void rejectsWhenThereAreNoObservations() {
    VisionResult result = run(new ArrayList<TagObservation>(), TRUE_POSE, true);
    assertEquals(Reason.NO_OBSERVATIONS, result.reason);
    assertFalse(result.accepted);
  }

  @Test
  public void rejectsAStaleFrame() {
    TagObservation tag = observe(31, Cell.AUDIENCE, TRUE_POSE); // Frame at T0 + 50 ms
    VisionResult stale = corrector.process(list(tag), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true,
        T0 + 50 * MS + 550 * MS);
    assertEquals(Reason.NO_VALID_TAGS, stale.reason);
    assertEquals(TagReject.FRAME_TOO_OLD, stale.firstTagReject);
    assertTrue(corrector.process(list(tag), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true,
        T0 + 50 * MS + 450 * MS).accepted);
  }

  @Test
  public void rejectsAFrameOlderThanTheHistory() {
    TagObservation tag = observe(LEFT, TRUE_POSE, 31, Cell.AUDIENCE, Double.NaN, T0 - 250 * MS);
    VisionResult result = corrector.process(list(tag), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true, T0);
    assertEquals(TagReject.NO_HISTORY, result.firstTagReject);
    assertFalse(result.accepted);
    VisionResult noHistory = corrector.process(list(observe(31, Cell.AUDIENCE, TRUE_POSE)),
        new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE), TRUE_POSE, 0, 0, 0, true, T0 + 100 * MS);
    assertEquals(TagReject.NO_HISTORY, noHistory.firstTagReject);
  }

  // ---- Working out the correction -----------------------------------------

  @Test
  public void takesTheMedianOfOddAndEvenCounts() {
    // Each tag is seen from a robot a little off the true pose, so its correction equals that offset.
    // Three tags: the median of {0, 0.02, 0.08} is 0.02. Tags from different poses would also turn the heading
    ShootOnTheMoveConstants.VISION_HEADING = false;
    VisionResult odd = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.00, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.02, 0.01)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.08, -0.01))), TRUE_POSE, true);
    assertTrue(odd.accepted);
    assertEquals(0.02, odd.dxM, MM_TOLERANCE);
    assertEquals(0.0, odd.dyM, MM_TOLERANCE);
    assertEquals(3, odd.tagsUsed);
    assertEquals(Math.hypot(0.06, 0.01), odd.spreadM, MM_TOLERANCE);
    assertEquals(3, odd.usedIds.length);

    // Four tags: the mean of the middle two
    VisionResult even = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.00, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.02, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.04, 0.0)),
        observe(35, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.06, 0.0))), TRUE_POSE, true);
    assertTrue(even.accepted);
    assertEquals(0.03, even.dxM, MM_TOLERANCE);
  }

  @Test
  public void rejectsWhenTheTagsDisagree() {
    VisionResult result = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.30, 0.0))), TRUE_POSE, true);
    assertEquals(Reason.SPREAD, result.reason);
    assertFalse(result.accepted);
    assertEquals(0.30, result.spreadM, MM_TOLERANCE);
    assertNull(result.newPose);

    ShootOnTheMoveConstants.MAX_SPREAD_M = 0.35; // A looser limit lets the same data through, at the median
    VisionResult loose = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.30, 0.0))), TRUE_POSE, true);
    assertTrue(loose.accepted);
    assertEquals(0.0, loose.dxM, MM_TOLERANCE);
  }

  @Test
  public void measuresTheCorrectionFromWhereTheRobotWasAtTheFrame() {
    // Odometry went from x = 0 to x = 1 (heading -pi/2 to -pi/2 + 0.2) over one second
    Pose start = new Pose(0.0, -0.9, -Math.PI / 2);
    Pose end = new Pose(1.0, -0.9, -Math.PI / 2 + 0.2);
    PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
    history.add(T0, start);
    history.add(T0 + 1000 * MS, end);
    // At the frame (0.5 s) odometry said (0.5, -0.9, -pi/2 + 0.1). The real robot was 0.10 m ahead in x
    // and 0.05 m ahead in y of that
    Pose truthAtFrame = new Pose(0.6, -0.85, -Math.PI / 2 + 0.1);
    TagObservation tag = observe(RIGHT, truthAtFrame, 35, Cell.AUDIENCE, Double.NaN, T0 + 500 * MS);
    VisionResult result = corrector.process(list(tag), history, end, 0.5, 0, 0, true, T0 + 600 * MS);
    assertTrue(result.accepted);
    assertEquals(0.10, result.dxM, MM_TOLERANCE);
    assertEquals(0.05, result.dyM, MM_TOLERANCE);
    assertEquals(1.05, result.newPose.xM, MM_TOLERANCE); // Half of it, applied to the current pose
    assertEquals(-0.875, result.newPose.yM, MM_TOLERANCE);
    assertEquals(end.headingRad, result.newPose.headingRad, TOLERANCE);
  }

  @Test
  public void theFirstFixAcceptsALargeJumpInFull() {
    Pose odometry = offsetBy(TRUE_POSE, 0.9, 0.7); // Far beyond MAX_JUMP_M
    VisionResult result = corrector.process(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), stillHistory(odometry),
        odometry, 0, 0, 0, false, true, T0 + 100 * MS); // Heading set by the driver (A)
    assertTrue(result.accepted);
    assertEquals(1.0, result.gain, TOLERANCE);
    assertEquals(-0.9, result.dxM, MM_TOLERANCE);
    assertEquals(-0.7, result.dyM, MM_TOLERANCE);
    assertEquals(TRUE_POSE.xM, result.newPose.xM, MM_TOLERANCE);
    assertEquals(TRUE_POSE.yM, result.newPose.yM, MM_TOLERANCE);
    assertEquals(-0.9, result.appliedDxM(), MM_TOLERANCE);
  }

  @Test
  public void theFirstFixStillNeedsAGoodFix() {
    VisionResult result = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.5, 0.0))), TRUE_POSE, false);
    assertEquals(Reason.SPREAD, result.reason);
    assertFalse(result.accepted);
  }

  @Test
  public void oneTagCannotPlaceTheRobotBeforeTheHeadingIsKnown() {
    // Logged 5 Nov (082826): start heading 154 degrees out, and tag 45 alone put the robot 2 m from the truth
    Pose odometry = new Pose(0, 0, TRUE_POSE.headingRad + Math.toRadians(154));
    VisionResult one = corrector.process(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), stillHistory(odometry),
        odometry, 0, 0, 0, false, false, T0 + 100 * MS);
    assertEquals(Reason.NO_HEADING, one.reason);
    assertFalse(one.accepted);
    // Two tags far enough apart fix the heading first, and then the position is right
    VisionResult pair = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0, 0, 0,
        false, false, T0 + 100 * MS);
    assertTrue(pair.accepted);
    assertEquals(TRUE_POSE.xM, pair.newPose.xM, MM_TOLERANCE);
    assertEquals(TRUE_POSE.yM, pair.newPose.yM, MM_TOLERANCE);
  }

  @Test
  public void oneTagPlacesTheRobotWhenItsHeadingWasSeededByPlacement() {
    // Shoot Auto, 13 Nov: one tag in view from its start spot, and the seeded pose 0.26 m off
    Pose odometry = new Pose(TRUE_POSE.xM - 0.26, TRUE_POSE.yM, TRUE_POSE.headingRad);
    corrector.setHeadingSeeded(true);
    VisionResult one = corrector.process(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), stillHistory(odometry),
        odometry, 0, 0, 0, false, false, T0 + 100 * MS);
    assertTrue(one.reason.toString(), one.accepted);
    assertEquals(TRUE_POSE.xM, one.newPose.xM, MM_TOLERANCE);
    corrector.reset();
    VisionResult unseeded = corrector.process(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), stillHistory(odometry),
        odometry, 0, 0, 0, false, false, T0 + 200 * MS);
    assertEquals(Reason.NO_HEADING, unseeded.reason);
  }

  @Test
  public void aBigJumpThatKeepsRepeatingReplacesATrustedPosition() {
    Pose odometry = offsetBy(TRUE_POSE, 1.8, 0.8);
    for (int frame = 1; frame < ShootOnTheMoveConstants.POSE_OVERRIDE_FRAMES; frame++) {
      assertEquals(Reason.JUMP, run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, true).reason);
    }
    VisionResult result = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, true);
    assertEquals(Reason.RELOCATED, result.reason);
    assertTrue(result.accepted);
    assertEquals(1.0, result.gain, TOLERANCE);
    assertEquals(TRUE_POSE.xM, result.newPose.xM, MM_TOLERANCE);
    assertEquals(TRUE_POSE.yM, result.newPose.yM, MM_TOLERANCE);
  }

  @Test
  public void bigJumpsThatDisagreeNeverReplaceATrustedPosition() {
    for (int frame = 0; frame < 3 * ShootOnTheMoveConstants.POSE_OVERRIDE_FRAMES; frame++) {
      Pose odometry = offsetBy(TRUE_POSE, frame % 2 == 0 ? 0.8 : -0.8, 0);
      assertEquals(Reason.JUMP, run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, true).reason);
    }
  }

  @Test
  public void rejectsAJumpOnceThePoseIsTrusted() {
    Pose far = offsetBy(TRUE_POSE, 0.6, 0.0);
    VisionResult result = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), far, true);
    assertEquals(Reason.JUMP, result.reason);
    assertFalse(result.accepted);
    Pose near = offsetBy(TRUE_POSE, 0.4, 0.0);
    assertTrue(run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), near, true).accepted);
  }

  @Test
  public void appliesTheGainOnceThePoseIsTrusted() {
    Pose odometry = offsetBy(TRUE_POSE, 0.2, -0.1);
    VisionResult result = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, true);
    assertTrue(result.accepted);
    assertEquals(0.5, result.gain, TOLERANCE);
    assertEquals(-0.2, result.dxM, MM_TOLERANCE);
    assertEquals(0.1, result.dyM, MM_TOLERANCE);
    assertEquals(odometry.xM - 0.1, result.newPose.xM, MM_TOLERANCE);
    assertEquals(odometry.yM + 0.05, result.newPose.yM, MM_TOLERANCE);
    assertEquals(odometry.headingRad, result.newPose.headingRad, TOLERANCE);
    assertEquals(-0.1, result.appliedDxM(), MM_TOLERANCE);
    assertEquals(0.05, result.appliedDyM(), MM_TOLERANCE);

    ShootOnTheMoveConstants.CORRECTION_GAIN = 0.25;
    VisionResult quarter = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, true);
    assertEquals(odometry.xM - 0.05, quarter.newPose.xM, MM_TOLERANCE);
  }

  @Test
  public void rejectsAPositionOutsideTheField() {
    Pose out = new Pose(1.75, -0.9, -Math.PI / 2); // The limit is 1.83 - 0.15 = 1.68 each way
    VisionResult result = run(list(observe(31, Cell.AUDIENCE, out)), out, true);
    assertEquals(Reason.OUT_OF_FIELD, result.reason);
    assertFalse(result.accepted);
    Pose outY = new Pose(0.1, 1.70, -Math.PI / 2);
    assertEquals(Reason.OUT_OF_FIELD, run(list(observe(31, Cell.AUDIENCE, outY)), outY, true).reason);
    Pose inside = new Pose(1.6, -0.9, -Math.PI / 2);
    assertTrue(run(list(observe(31, Cell.AUDIENCE, inside)), inside, true).accepted);
    assertEquals(Reason.OUT_OF_FIELD, run(list(observe(31, Cell.AUDIENCE, out)), out, false).reason); // Still checked for the first fix
  }

  // ---- Heading ----------------------------------------------------------

  private VisionResult runHeading(List<TagObservation> observations, Pose odometry, boolean headingValid) {
    return corrector.process(observations, stillHistory(odometry), odometry, 0, 0, 0, true, headingValid,
        T0 + 100 * MS);
  }

  private static List<TagObservation> fourRedTags(Pose truth) {
    return list(observe(30, Cell.AUDIENCE, truth), observe(33, Cell.AUDIENCE, truth),
        observe(34, Cell.AUDIENCE, truth), observe(37, Cell.AUDIENCE, truth));
  }

  @Test
  public void twoTagsFixAWrongStartHeadingInFull() {
    // Placed 25 degrees off square, and odometry also 0.1 m out
    Pose odometry = new Pose(TRUE_POSE.xM + 0.1, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(25));
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, false);
    assertTrue(result.accepted);
    assertEquals(1.0, result.headingGain, TOLERANCE);
    assertEquals(TRUE_POSE.headingRad, result.newPose.headingRad, 1e-6);
    // With the heading fixed, the tags agree on the position too
    assertTrue(result.spreadM < MM_TOLERANCE);
    assertEquals(-0.1, result.dxM, MM_TOLERANCE);
  }

  @Test
  public void aTrustedHeadingIsTrimmedByTheGain() {
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(5));
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, true);
    assertTrue(result.accepted);
    assertEquals(Math.toRadians(-5), result.dHeadingRad, 1e-6);
    assertEquals(ShootOnTheMoveConstants.HEADING_GAIN, result.headingGain, TOLERANCE);
    assertEquals(odometry.headingRad + ShootOnTheMoveConstants.HEADING_GAIN * Math.toRadians(-5),
        result.newPose.headingRad, 1e-6);
  }

  @Test
  public void tagsThatOnlyDisagreeThroughTheHeadingFixItInFull() {
    ShootOnTheMoveConstants.MAX_SPREAD_M = 0.03; // Tight enough that a 15 degree heading error spreads them
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(15));
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, true);
    assertTrue(result.accepted);
    assertEquals(1.0, result.headingGain, TOLERANCE);
    assertEquals(TRUE_POSE.headingRad, result.newPose.headingRad, 1e-6);
    assertTrue(result.spreadM < MM_TOLERANCE);

    VisionResult moving = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0.4, 0, 0,
        true, true, T0 + 100 * MS);
    assertEquals(Reason.SPREAD, moving.reason); // Pair headings taken moving are not trusted
  }

  @Test
  public void movingTheTrustedHeadingIsLeftToOdometry() {
    // 5 Nov: a frame taken at 1 m/s turning 39 deg/s put the heading 11.5 degrees out, where odometry was right
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(5));
    VisionResult moving = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0.4, 0, 0,
        true, true, T0 + 100 * MS);
    assertTrue(moving.accepted); // The position still is
    assertEquals(0, moving.headingGain, TOLERANCE);
    VisionResult turning = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0, 0, 0.5,
        true, true, T0 + 100 * MS);
    assertEquals(0, turning.headingGain, TOLERANCE);
    // Not yet trusted, the first heading fix is still taken in full
    VisionResult first = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0.4, 0, 0,
        true, false, T0 + 100 * MS);
    assertEquals(1.0, first.headingGain, TOLERANCE);
  }

  @Test
  public void aBigHeadingJumpIsIgnoredOnceTrusted() {
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(40));
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, true);
    assertEquals(0, result.headingGain, TOLERANCE);
    assertTrue(result.newPose == null || result.newPose.headingRad == odometry.headingRad);
  }

  @Test
  public void aBigHeadingFixThatKeepsRepeatingReplacesATrustedHeading() {
    // Logged 4 Nov: a bad Pinpoint read left the trusted heading 172 degrees out, and the tags kept saying so
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(172));
    for (int frame = 1; frame < ShootOnTheMoveConstants.HEADING_OVERRIDE_FRAMES; frame++) {
      assertEquals(0, runHeading(fourRedTags(TRUE_POSE), odometry, true).headingGain, TOLERANCE);
    }
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, true);
    assertEquals(1.0, result.headingGain, TOLERANCE);
    assertTrue(result.accepted);
    assertEquals(TRUE_POSE.headingRad, result.newPose.headingRad, 1e-6);
  }

  @Test
  public void bigHeadingFixesThatDisagreeNeverReplaceATrustedHeading() {
    Pose plus = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(90));
    Pose minus = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad - Math.toRadians(90));
    for (int frame = 0; frame < 3 * ShootOnTheMoveConstants.HEADING_OVERRIDE_FRAMES; frame++) {
      Pose odometry = frame % 2 == 0 ? plus : minus;
      assertEquals(0, runHeading(fourRedTags(TRUE_POSE), odometry, true).headingGain, TOLERANCE);
    }
  }

  @Test
  public void theFirstHeadingWaitsForEnoughTagPairs() {
    Pose odometry = new Pose(0, 0, TRUE_POSE.headingRad + Math.toRadians(28));
    VisionResult pair = corrector.process(list(observe(30, Cell.AUDIENCE, TRUE_POSE), observe(37, Cell.AUDIENCE, TRUE_POSE)),
        stillHistory(odometry), odometry, 0, 0, 0, false, false, T0 + 100 * MS);
    assertEquals(1, pair.headingPairs);
    assertEquals(0, pair.headingGain, TOLERANCE);
    assertFalse(pair.accepted);
    VisionResult four = corrector.process(fourRedTags(TRUE_POSE), stillHistory(odometry), odometry, 0, 0, 0,
        false, false, T0 + 100 * MS);
    assertTrue(four.headingPairs >= ShootOnTheMoveConstants.MIN_FIRST_HEADING_PAIRS);
    assertTrue(four.accepted);
    assertEquals(TRUE_POSE.headingRad, four.newPose.headingRad, 1e-6);
  }

  @Test
  public void oneTagOrTagsTooCloseLeaveTheHeadingAlone() {
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(3));
    VisionResult one = runHeading(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, false);
    assertEquals(0, one.headingPairs);
    assertEquals(odometry.headingRad, one.newPose.headingRad, TOLERANCE);
    VisionResult close = runHeading(list(observe(30, Cell.AUDIENCE, TRUE_POSE),
        observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, false); // 3.75 inches apart
    assertEquals(0, close.headingPairs);
  }

  @Test
  public void anOffFieldPositionStillFixesTheHeading() {
    // Logged 4 Nov: placed facing the driver (180 degrees off), and the tags put it past the field edge
    Pose truth = new Pose(-1.75, -0.5, Math.PI);
    Pose odometry = new Pose(0, 0, 0);
    ShootOnTheMoveConstants.FIELD_HALF_SIZE_M = 1.83;
    ShootOnTheMoveConstants.ROBOT_HALF_SIZE_M = 0.15;
    VisionResult result = corrector.process(fourRedTags(truth), stillHistory(odometry), odometry, 0, 0, 0,
        false, false, T0 + 100 * MS);
    assertEquals(Reason.HEADING_ONLY, result.reason);
    assertTrue(result.accepted);
    assertEquals(0, result.gain, TOLERANCE); // Position left alone
    assertEquals(0, result.newPose.xM, TOLERANCE);
    assertEquals(Math.PI, Math.abs(result.newPose.headingRad), 1e-6);
  }

  private VisionResult initialPair(Pose truth, Pose odometry, long elapsedMs) {
    return initialPair(truth, odometry, elapsedMs, 0, 36, 39);
  }

  private VisionResult initialPair(Pose truth, Pose odometry, long elapsedMs, double speed, int a, int b) {
    long frame = T0 + elapsedMs * MS;
    PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
    history.add(frame - MS, odometry); history.add(frame + MS, odometry);
    return corrector.process(list(observe(LEFT, truth, a, Cell.AUDIENCE, Double.NaN, frame),
        observe(RIGHT, truth, b, Cell.AUDIENCE, Double.NaN, frame)),
        history, odometry, speed, 0, 0, false, false, frame + MS);
  }

  @Test public void aWidePairCanBootstrapFromFourFreshConsistentFrames() {
    Pose odometry = new Pose(0, 0, TRUE_POSE.headingRad + Math.toRadians(134));
    for (int i = 0; i < 3; i++) {
      assertFalse(initialPair(TRUE_POSE, odometry, 100 + i * 500).accepted);
    }
    VisionResult fixed = initialPair(TRUE_POSE, odometry, 1600);
    assertTrue(fixed.accepted); assertEquals(1, fixed.headingGain, TOLERANCE);
    assertEquals(TRUE_POSE.headingRad, fixed.newPose.headingRad, 1e-6);
    assertEquals(TRUE_POSE.xM, fixed.newPose.xM, 1e-6);
    assertEquals(TRUE_POSE.yM, fixed.newPose.yM, 1e-6);
  }

  @Test public void replayingTheSamePairFrameCannotBootstrap() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 8; i++) { assertFalse(initialPair(TRUE_POSE, odometry, 100).accepted); }
    assertFalse(initialPair(TRUE_POSE, odometry, 600).accepted);
    assertFalse(initialPair(TRUE_POSE, odometry, 1100).accepted);
    assertTrue(initialPair(TRUE_POSE, odometry, 1600).accepted);
  }

  @Test public void staleEvidenceAndResetCannotCompleteAFirstHeading() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 3; i++) { initialPair(TRUE_POSE, odometry, 100 + i * 500); }
    assertFalse(initialPair(TRUE_POSE, odometry, 4000).accepted);
    assertFalse(initialPair(TRUE_POSE, odometry, 4500).accepted);
    assertFalse(initialPair(TRUE_POSE, odometry, 5000).accepted);
    corrector.reset();
    assertFalse(initialPair(TRUE_POSE, odometry, 5500).accepted);
  }

  @Test public void inconsistentPairHeadingsOrPositionsRestartConfirmation() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 3; i++) { initialPair(TRUE_POSE, odometry, 100 + i * 500); }
    Pose rotated = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(10));
    assertFalse(initialPair(rotated, odometry, 1600).accepted);
    assertFalse(initialPair(TRUE_POSE, odometry, 2100).accepted);
    assertFalse(initialPair(TRUE_POSE, odometry, 2600).accepted);
    Pose moved = new Pose(TRUE_POSE.xM + 0.2, TRUE_POSE.yM, TRUE_POSE.headingRad);
    assertFalse(initialPair(moved, odometry, 3100).accepted);
  }

  @Test public void coincidentPairAndMovingPairCannotUseTemporalBootstrap() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 8; i++) {
      assertFalse(initialPair(TRUE_POSE, odometry, 100 + i * 500, 0, 30, 30).accepted);
    }
    for (int i = 0; i < 8; i++) {
      assertFalse(initialPair(TRUE_POSE, odometry, 5000 + i * 500, 0.4, 36, 39).accepted);
    }
  }

  @Test public void loggedNovemberNineTwoTagViewBootstrapsAfterRepeatedAgreement() {
    // Raw right-camera detections from VolleyTuning 20261109_133512, first four pair frames.
    double[][] samples = {
      {4695962113061.0, 4695767604042L, -0.00281684249651443, 0.0017286517499545925, 1.505723936472043, -0.002816800594329834, 0.0017292962074279788, 1.5057149092343192, 5.869425949640577e-07, 9.204228408634663e-06, -0.00012899182636066228, -0.2242087951660156, 1.2989484268188476, -0.28127495822906495, 44.146461486816406, -0.7187237655639648, 1.3777570854187011, -0.20047271976470946, 57.351806640625},
      {4696700250594.0, 4696531311410L, -0.002816868912899357, 0.0017282249702883388, 1.5057299263320074, -0.002816685914993286, 0.0017311090230941774, 1.5056895176556448, -1.9434280693531058e-05, -0.0003045870363712311, 0.004268617324537458, -0.2236796365737915, 1.296077187347412, -0.28049947261810304, 45.23434829711914, -0.7171062770843506, 1.3747081436157227, -0.2001549097061157, 60.73168182373047},
      {4697480674547.0, 4697311374863L, -0.002815975824770538, 0.001742196794312798, 1.5055341312442012, -0.0028156647682189943, 0.0017470703125000001, 1.505465836324964, -3.179271519184115e-05, -0.0004965097904205322, 0.006958407684848562, -0.22419576301574706, 1.2990519088745116, -0.2813400463104248, 44.949073791503906, -0.7167881763458251, 1.3741814315795897, -0.20002628383636473, 57.8216552734375},
      {4698882156355.0, 4698722357130L, -0.004476212775325933, 0.002974521803718562, 1.4592086076818764, -0.00447624683380127, 0.002974218606948853, 1.45921287039784, -8.46834154799581e-07, -7.654430344700813e-06, 0.00010770802628114828, -0.17675920839309692, 1.3040672035217284, -0.2779656398773193, 41.55519485473633, -0.6622377754211426, 1.3946982994079589, -0.18270280151367188, 52.12730026245117},
    };
    VisionResult result = null;
    for (int i = 0; i < samples.length; i++) {
      double[] row = samples[i];
      long now = (long) row[0], frame = (long) row[1];
      Pose atFrame = new Pose(row[2], row[3], row[4]);
      Pose odometry = new Pose(row[5], row[6], row[7]);
      PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
      history.add(frame - MS, atFrame); history.add(frame + MS, atFrame);
      result = corrector.process(list(new TagObservation(36, RIGHT, row[11], row[12], row[13], row[14], frame),
          new TagObservation(39, RIGHT, row[15], row[16], row[17], row[18], frame)),
          history, odometry, row[8], row[9], row[10], false, false, now);
      if (i < 3) { assertFalse(result.accepted); }
    }
    assertEquals(Reason.ACCEPTED, result.reason);
    assertTrue(corrector.upCellConfirmed(Alliance.BLUE));
    assertEquals(1, result.headingGain, TOLERANCE);
    assertTrue(result.spreadM < 0.05);
  }

  @Test public void loggedAdjacentTagViewBootstrapsAfterRepeatedAgreement() {
    // Raw left-camera detections from VolleyTuning 20261109_135640, first four pair frames.
    double[][] samples = {
      {6007364592616.0, 6007151535928L, -0.020057160161133708, -0.044437067188100754, 2.579309263161799, -0.020053493499755864, -0.044439403533935544, 2.579374031224523, -1.4060030691325665e-05, 8.83099064230919e-06, -0.00023221428573645228, 0.4218749759674072, 1.2177379768371581, -0.3691796459197998, 27.63983726501465, 0.29111958503723145, 1.1784482238769531, -0.400250036239624, 31.083492279052734},
      {6008686740791.0, 6008509448565L, -0.02006179585564588, -0.04443426229789407, 2.579244140228688, -0.0200640983581543, -0.044432815551757814, 2.57920761505631, 0.0002345646470785141, -0.0001473861932754517, 0.0038744819667733406, 0.4225403423309326, 1.2198320350646972, -0.3698909397125244, 28.40828514099121, 0.2916225824356079, 1.1803638046264648, -0.4007834581375122, 28.86903190612793},
      {6008982439779.0, 6008768435757L, -0.02006271609571001, -0.04443368617597474, 2.579232402545142, -0.020071798324584965, -0.044427986145019534, 2.5790841142323355, -0.0005429808497428894, 0.0003412488400936127, -0.00896936634560255, 0.422960326385498, 1.2210745948791504, -0.3702558399200439, 26.914892196655273, 0.2909491012573242, 1.1782230438232422, -0.40019134311676025, 28.871126174926758},
      {6009238225929.0, 6009055467869L, -0.02007336613863901, -0.044427015220929476, 2.5790571262691957, -0.020029365539550786, -0.044454730987548825, 2.5790745774891715, -6.140373274683952e-05, 3.8590084761381157e-05, -0.0010143093088477062, 0.42135184898376465, 1.216073931121826, -0.3684538423538208, 28.66648292541504, 0.2916223402023315, 1.1808970085144042, -0.40104107322692867, 29.309232711791992},
    };
    VisionResult result = null;
    for (int i = 0; i < samples.length; i++) {
      double[] row = samples[i];
      long now = (long) row[0], frame = (long) row[1];
      Pose atFrame = new Pose(row[2], row[3], row[4]);
      Pose odometry = new Pose(row[5], row[6], row[7]);
      PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
      history.add(frame - MS, atFrame); history.add(frame + MS, atFrame);
      result = corrector.process(list(new TagObservation(35, LEFT, row[11], row[12], row[13], row[14], frame),
          new TagObservation(36, LEFT, row[15], row[16], row[17], row[18], frame)),
          history, odometry, row[8], row[9], row[10], false, false, now);
      if (i < 3) { assertFalse(result.accepted); }
    }
    assertEquals(Reason.ACCEPTED, result.reason);
    assertEquals(1, result.headingGain, TOLERANCE);
    assertTrue(result.spreadM < 0.05);
  }

  @Test public void adjacentPairsNeedFourFramesAndDoNotChangeTrustedHeadingRules() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 3; i++) {
      assertFalse(initialPair(TRUE_POSE, odometry, 100 + i * 500, 0, 30, 31).accepted);
    }
    VisionResult fixed = initialPair(TRUE_POSE, odometry, 1600, 0, 30, 31);
    assertEquals(Reason.ACCEPTED, fixed.reason);
    assertEquals(TRUE_POSE.headingRad, fixed.newPose.headingRad, 1e-6);
    Pose trusted = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(3));
    for (int i = 0; i < 8; i++) {
      VisionResult trim = runHeading(list(observe(30, Cell.AUDIENCE, TRUE_POSE),
          observe(31, Cell.AUDIENCE, TRUE_POSE)), trusted, true);
      assertEquals(0, trim.headingPairs); assertEquals(0, trim.headingGain, TOLERANCE);
    }
  }

  @Test public void temporalBootstrapHonorsDisabledVisionHeading() {
    ShootOnTheMoveConstants.VISION_HEADING = false;
    for (int i = 0; i < 8; i++) {
      assertFalse(initialPair(TRUE_POSE, new Pose(0, 0, 0), 100 + i * 500).accepted);
    }
  }

  @Test public void aClosePairWithInconsistentRangeCannotBootstrapEvenWhenRepeated() {
    Pose odometry = new Pose(0, 0, 0);
    for (int i = 0; i < 8; i++) {
      long frame = T0 + (100 + i * 500) * MS;
      PoseHistory history = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
      history.add(frame - MS, odometry); history.add(frame + MS, odometry);
      TagObservation a = observe(LEFT, TRUE_POSE, 35, Cell.AUDIENCE, Double.NaN, frame);
      TagObservation b = observe(LEFT, TRUE_POSE, 36, Cell.AUDIENCE, Double.NaN, frame);
      TagObservation badRange = new TagObservation(b.id, b.camera, b.rightM,
          b.forwardM + 0.10, b.upM, b.decisionMargin, frame);
      VisionResult result = corrector.process(list(a, badRange), history, odometry, 0, 0, 0,
          false, false, frame + MS);
      assertEquals(2, result.tagsUsed); assertEquals(TagReject.NONE, result.firstTagReject);
      assertFalse(result.accepted); assertEquals(0, result.headingGain, TOLERANCE);
    }
  }

}
