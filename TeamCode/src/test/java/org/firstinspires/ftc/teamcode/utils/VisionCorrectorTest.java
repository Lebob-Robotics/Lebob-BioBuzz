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
    // The left camera looks backward and to the left, so its picture-right points forward and left
    double[] ahead = LEFT.toRobotFrame(0, 1, 0);
    assertTrue(ahead[0] < LEFT.forwardM && ahead[1] > LEFT.leftM && ahead[2] > LEFT.heightM);
    double[] right = LEFT.toRobotFrame(1, 0, 0);
    assertTrue(right[0] > LEFT.forwardM && right[1] > LEFT.leftM);
    assertEquals(LEFT.heightM, right[2], TOLERANCE);
    double[] aheadRight = RIGHT.toRobotFrame(0, 1, 0); // The right camera is the mirror image
    assertTrue(aheadRight[0] < RIGHT.forwardM && aheadRight[1] < RIGHT.leftM && aheadRight[2] > RIGHT.heightM);
    double[] up = LEFT.toRobotFrame(0, 0, 1); // Picture-up raises the point by cos(pitch) per metre
    assertEquals(LEFT.heightM + Math.cos(LEFT.pitchRad), up[2], TOLERANCE);
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
      } else {
        assertEquals(Cell.SCORING, corrector.upCell(Alliance.RED));
        assertTrue(result.redFlipped);
        assertFalse(result.blueFlipped);
      }
    }
    assertEquals(Cell.SCORING, corrector.upCell(Alliance.BLUE)); // Untouched
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
    VisionResult fast = corrector.process(observations, history, TRUE_POSE, 0.8, 0.8, 0, true, T0 + 100 * MS);
    assertEquals(Reason.TOO_FAST, fast.reason);
    assertFalse(fast.accepted);
    assertTrue(corrector.process(observations, history, TRUE_POSE, 1.0, 0, 0, true, T0 + 100 * MS).accepted);
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
        T0 + 50 * MS + 350 * MS);
    assertEquals(Reason.NO_VALID_TAGS, stale.reason);
    assertEquals(TagReject.FRAME_TOO_OLD, stale.firstTagReject);
    assertTrue(corrector.process(list(tag), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true,
        T0 + 50 * MS + 250 * MS).accepted);
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
    VisionResult result = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odometry, false);
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
  public void aBigHeadingJumpIsIgnoredOnceTrusted() {
    Pose odometry = new Pose(TRUE_POSE.xM, TRUE_POSE.yM, TRUE_POSE.headingRad + Math.toRadians(40));
    VisionResult result = runHeading(fourRedTags(TRUE_POSE), odometry, true);
    assertEquals(0, result.headingGain, TOLERANCE);
    assertTrue(result.newPose == null || result.newPose.headingRad == odometry.headingRad);
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
}
