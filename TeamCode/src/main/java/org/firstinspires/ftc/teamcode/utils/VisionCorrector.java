package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants.*;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.utils.VisionResult.Reason;
import org.firstinspires.ftc.teamcode.utils.VisionResult.TagReject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Keeps the odometry pose honest from the Hive's AprilTags, and tracks which Cell of each Hive is up.
 * Ported from Sean's VisionCorrector on feature/sotm. It only works out a correction: the caller
 * applies it, and keeps everything stored in the odometry frame in step.
 *
 * <p>The odometry heading must be in the field frame, because the tags have field positions. x and y
 * are corrected from every tag; the heading is corrected when two tags far enough apart are seen at
 * once, from the line between them, so the driver no longer has to reset it by hand. Robot frame is x forward, y left,
 * z up. Keep one corrector for the OpMode, since it holds the tip votes.
 */
public final class VisionCorrector {
  /** One end of a Hive. Exactly one of the two Cells is up at a time. */
  public enum Cell { AUDIENCE, SCORING }

  private Cell upCellRed = Cell.AUDIENCE; // Starts red AUDIENCE, blue SCORING. Vision flips them
  private Cell upCellBlue = Cell.SCORING;
  private final int[] flipVotes = new int[2]; // Frames in a row voting against the current up Cell: red, blue
  private final boolean[] upCellConfirmed = new boolean[2]; // Tags have shown which Cell is up: red, blue
  private int headingDisagreeFrames = 0; // Frames in a row whose heading fix was too big to trust, and agreed
  private double lastDisagreeRad = 0;
  private int jumpAgreeFrames = 0; // Frames whose position fix was too big to trust, each agreeing with the last
  private double lastJumpDxM = 0;
  private double lastJumpDyM = 0;

  /** The up Cell of an alliance's Hive. */
  public Cell upCell(Alliance alliance) {
    return alliance == Alliance.RED ? upCellRed : upCellBlue;
  }

  /** False until the tags' heights have shown which Cell of the Hive is up, not just the starting guess. */
  public boolean upCellConfirmed(Alliance alliance) {
    return upCellConfirmed[alliance == Alliance.RED ? 0 : 1];
  }

  /**
   * Where a Cell sighting puts a Hive's up Cell on the field, {x, y} in metres: the centre of its line of tags,
   * moved out from the Hive by the offset the sightings use (CELL_REF_UP_TOWARD_IN). Cell sightings agreed with
   * it within 4 cm (5 Nov, 081558).
   */
  public static double[] upCellAimPoint(Alliance alliance, Cell cell) {
    double ySumIn = 0;
    int tags = 0;
    for (int id = 30; id <= 45; id++) {
      if (tagAlliance(id) == alliance && tagCell(id) == cell) {
        ySumIn += TAG_Y_IN[id - 30];
        tags++;
      }
    }
    double outM = CLUSTER_UP_X_M + CELL_REF_UP_TOWARD_IN * IN_TO_M;
    return new double[] {cell == Cell.AUDIENCE ? outM : -outM, ySumIn / tags * IN_TO_M};
  }

  /** Clears partial tip votes. */
  public void reset() {
    flipVotes[0] = 0;
    flipVotes[1] = 0;
  }

  private static double spreadOf(double[] dx, double[] dy, double medianDx, double medianDy) {
    double spread = 0;
    for (int i = 0; i < dx.length; i++) {
      spread = Math.max(spread, Math.hypot(dx[i] - medianDx, dy[i] - medianDy));
    }
    return spread;
  }

  // ---- Tag field ---------------------------------------------------------

  /** The alliance a tag belongs to, or null for an id that is not a Hive tag. */
  public static Alliance tagAlliance(int id) {
    if (id >= 30 && id <= 37) {
      return Alliance.RED;
    }
    if (id >= 38 && id <= 45) {
      return Alliance.BLUE;
    }
    return null;
  }

  /** The Cell a tag sits on, or null for an id that is not a Hive tag. */
  public static Cell tagCell(int id) {
    if ((id >= 30 && id <= 33) || (id >= 42 && id <= 45)) {
      return Cell.SCORING;
    }
    if (id >= 34 && id <= 41) {
      return Cell.AUDIENCE;
    }
    return null;
  }

  public static Cell otherCell(Cell cell) {
    return cell == Cell.AUDIENCE ? Cell.SCORING : Cell.AUDIENCE;
  }

  /**
   * The field position of a tag centre.
   *
   * @param upCellOfThatHive which Cell of the tag's own Hive is up
   * @return {x, y, z} in metres, or null for an id that is not a Hive tag
   */
  public static double[] tagFieldPosition(int id, Cell upCellOfThatHive) {
    Alliance alliance = tagAlliance(id);
    Cell cell = tagCell(id);
    if (alliance == null || cell == null) {
      return null;
    }
    double yM = TAG_Y_IN[id - 30] * IN_TO_M;
    boolean up = cell == upCellOfThatHive;
    double xM = up ? CLUSTER_UP_X_M : CLUSTER_DOWN_X_M;
    double zM = up ? CLUSTER_UP_Z_M : CLUSTER_DOWN_Z_M;
    return new double[] {cell == Cell.AUDIENCE ? xM : -xM, yM, zM};
  }

  // ---- Correction --------------------------------------------------------

  /**
   * Works out one loop's pose correction. The tip state uses only the tags' heights, so it updates
   * whenever the robot is slow enough, even when the position correction is rejected.
   *
   * @param observations tags from frames not seen before. Never hand the same frame in twice
   * @param history odometry poses by time, with earlier corrections applied
   * @param now the current odometry pose
   * @param poseValid whether the pose is already trusted. False means this is the first fix, which
   *     may be any size and is applied in full
   * @param nowNanos the loop time, on the same clock as the frame times
   */
  public VisionResult process(List<TagObservation> observations, PoseHistory history, Pose now,
      double velocityXMps, double velocityYMps, double turnRateRadPerSec, boolean poseValid,
      long nowNanos) {
    return process(observations, history, now, velocityXMps, velocityYMps, turnRateRadPerSec, poseValid,
        poseValid, nowNanos);
  }

  /**
   * As above, with the heading trusted separately: false means the next heading fix may be any size
   * and is applied in full.
   */
  public VisionResult process(List<TagObservation> observations, PoseHistory history, Pose now,
      double velocityXMps, double velocityYMps, double turnRateRadPerSec, boolean poseValid,
      boolean headingValid, long nowNanos) {
    if (observations == null || observations.isEmpty()) {
      return fail(Reason.NO_OBSERVATIONS, TagReject.NONE, 0, false, false);
    }
    if (Math.hypot(velocityXMps, velocityYMps) > MAX_SPEED_FOR_VISION_MPS) {
      return fail(Reason.TOO_FAST, TagReject.NONE, 0, false, false);
    }
    if (Math.abs(turnRateRadPerSec) > MAX_OMEGA_FOR_VISION_RAD_S) {
      return fail(Reason.TOO_FAST_TURNING, TagReject.NONE, 0, false, false);
    }

    int[][] votes = new int[2][2]; // [Hive][Cell]: tags with a believable height saying that Cell is up
    List<double[]> usedTags = new ArrayList<>(); // Per tag: field x, y; offset from robot in the field frame; robot x, y at the frame
    List<Integer> ids = new ArrayList<>();
    TagReject firstReject = TagReject.NONE;
    int rejected = 0;

    for (TagObservation tag : observations) {
      TagReject reject = TagReject.NONE;
      Cell measuredUp = null;
      double[] inRobot = null;
      double[] tagField = null;
      Pose atFrame = null;

      if (Double.isNaN(tag.rightM) || Double.isNaN(tag.forwardM) || Double.isNaN(tag.upM)) {
        reject = TagReject.NO_POSE;
      } else if (tagAlliance(tag.id) == null) {
        reject = TagReject.NOT_HIVE_TAG;
      } else if (!(tag.decisionMargin >= MIN_DECISION_MARGIN)) {
        reject = TagReject.LOW_MARGIN;
      } else if (tag.rangeM() > MAX_TAG_RANGE_M) {
        reject = TagReject.TOO_FAR;
      } else {
        inRobot = tag.camera.toRobotFrame(tag.rightM, tag.forwardM, tag.upM);
        double heightM = inRobot[2]; // The robot is on the floor, so this is the field height too
        Cell cell = tagCell(tag.id);
        if (Math.abs(heightM - CLUSTER_UP_Z_M) < Z_TOL_M) {
          measuredUp = cell;
        } else if (Math.abs(heightM - CLUSTER_DOWN_Z_M) < Z_TOL_M) {
          measuredUp = otherCell(cell);
        } else {
          reject = TagReject.BAD_Z;
        }
      }

      if (reject == TagReject.NONE) {
        int hive = tagAlliance(tag.id) == Alliance.RED ? 0 : 1;
        votes[hive][measuredUp.ordinal()]++;
        tagField = tagFieldPosition(tag.id, measuredUp); // Placed using the tip state this tag measured
        double ageS = (nowNanos - tag.frameTimeNanos) * 1e-9;
        if (ageS > MAX_FRAME_AGE_S) {
          reject = TagReject.FRAME_TOO_OLD;
        } else {
          atFrame = history == null ? null : history.sample(tag.frameTimeNanos);
          if (atFrame == null) {
            reject = TagReject.NO_HISTORY;
          }
        }
      }

      if (reject != TagReject.NONE) {
        rejected++;
        if (firstReject == TagReject.NONE) {
          firstReject = reject;
        }
        continue;
      }

      // The tag's offset from the robot, turned into the field frame by the odometry heading at the frame
      double cosHeading = Math.cos(atFrame.headingRad);
      double sinHeading = Math.sin(atFrame.headingRad);
      double offsetX = inRobot[0] * cosHeading - inRobot[1] * sinHeading;
      double offsetY = inRobot[0] * sinHeading + inRobot[1] * cosHeading;
      usedTags.add(new double[] {tagField[0], tagField[1], offsetX, offsetY, atFrame.xM, atFrame.yM});
      ids.add(tag.id);
    }

    // Heading: for each pair of tags, the turn that lines up where odometry puts them with the field
    List<Double> headingFixes = new ArrayList<>();
    for (int i = 0; VISION_HEADING && i < usedTags.size(); i++) {
      for (int j = i + 1; j < usedTags.size(); j++) {
        double[] a = usedTags.get(i);
        double[] b = usedTags.get(j);
        double fieldX = b[0] - a[0];
        double fieldY = b[1] - a[1];
        if (Math.hypot(fieldX, fieldY) < MIN_HEADING_BASELINE_M) {
          continue;
        }
        double seenX = (b[4] + b[2]) - (a[4] + a[2]);
        double seenY = (b[5] + b[3]) - (a[5] + a[3]);
        headingFixes.add(ShootOnTheMove.normalizeAngle(Math.atan2(fieldY, fieldX) - Math.atan2(seenY, seenX)));
      }
    }
    double dHeading = 0;
    double headingGain = 0;
    boolean steadyForHeading = Math.hypot(velocityXMps, velocityYMps) <= MAX_SPEED_FOR_VISION_HEADING_MPS
        && Math.abs(turnRateRadPerSec) <= MAX_OMEGA_FOR_VISION_HEADING_RAD_S;
    boolean enoughPairs = headingValid || headingFixes.size() >= MIN_FIRST_HEADING_PAIRS;
    if (!headingFixes.isEmpty() && enoughPairs && (steadyForHeading || !headingValid)) {
      double[] fixes = new double[headingFixes.size()];
      for (int k = 0; k < fixes.length; k++) {
        fixes[k] = headingFixes.get(k);
      }
      dHeading = median(fixes);
      boolean believable = !headingValid || Math.abs(dHeading) <= Math.toRadians(MAX_HEADING_JUMP_DEG);
      boolean override = false;
      if (believable) {
        headingDisagreeFrames = 0;
      } else {
        // A big fix the tags keep giving means odometry has lost its heading, so take it once it repeats
        boolean agrees = headingDisagreeFrames > 0 && Math.abs(ShootOnTheMove.normalizeAngle(dHeading - lastDisagreeRad))
            <= Math.toRadians(HEADING_OVERRIDE_AGREE_DEG);
        headingDisagreeFrames = agrees ? headingDisagreeFrames + 1 : 1;
        lastDisagreeRad = dHeading;
        override = headingDisagreeFrames >= HEADING_OVERRIDE_FRAMES; // Stays on until a fix is believable again
      }
      headingGain = override || !headingValid ? 1.0 : believable ? HEADING_GAIN : 0;
    }

    // Robot position is the tag position less its offset, turned by the heading correction about the robot
    double cosFix = Math.cos(headingGain * dHeading);
    double sinFix = Math.sin(headingGain * dHeading);
    List<Double> dxs = new ArrayList<>();
    List<Double> dys = new ArrayList<>();
    for (double[] t : usedTags) {
      dxs.add(t[0] - (t[2] * cosFix - t[3] * sinFix) - t[4]);
      dys.add(t[1] - (t[2] * sinFix + t[3] * cosFix) - t[5]);
    }

    boolean redFlipped = applyVotes(0, votes[0], Alliance.RED);
    boolean blueFlipped = applyVotes(1, votes[1], Alliance.BLUE);

    int used = dxs.size();
    if (used == 0) {
      return fail(Reason.NO_VALID_TAGS, firstReject, rejected, redFlipped, blueFlipped);
    }
    double[] usedDx = new double[used];
    double[] usedDy = new double[used];
    int[] usedIds = new int[used];
    for (int i = 0; i < used; i++) {
      usedDx[i] = dxs.get(i);
      usedDy[i] = dys.get(i);
      usedIds[i] = ids.get(i);
    }
    double medianDx = median(usedDx);
    double medianDy = median(usedDy);
    double spread = spreadOf(usedDx, usedDy, medianDx, medianDy);
    if (spread > MAX_SPREAD_M && headingGain > 0 && headingGain < 1.0 && steadyForHeading && used >= 3) {
      // The tags only disagree because the heading is wrong if turning by all of the pairs' heading brings them
      // together. Standing still, five tags agreed the heading was 13 degrees out and every frame was refused for
      // spread, so the heading was never fixed (5 Nov, 080149). Jumps too big to believe still wait for the override
      double cosAll = Math.cos(dHeading);
      double sinAll = Math.sin(dHeading);
      double[] turnedDx = new double[used];
      double[] turnedDy = new double[used];
      for (int i = 0; i < used; i++) {
        double[] t = usedTags.get(i);
        turnedDx[i] = t[0] - (t[2] * cosAll - t[3] * sinAll) - t[4];
        turnedDy[i] = t[1] - (t[2] * sinAll + t[3] * cosAll) - t[5];
      }
      double turnedMedianDx = median(turnedDx);
      double turnedMedianDy = median(turnedDy);
      double turnedSpread = spreadOf(turnedDx, turnedDy, turnedMedianDx, turnedMedianDy);
      if (turnedSpread <= MAX_SPREAD_M) {
        headingGain = 1.0;
        headingDisagreeFrames = 0;
        usedDx = turnedDx;
        usedDy = turnedDy;
        medianDx = turnedMedianDx;
        medianDy = turnedMedianDy;
        spread = turnedSpread;
      }
    }

    double limitM = FIELD_HALF_SIZE_M - ROBOT_HALF_SIZE_M; // Where vision says the robot is now must be on the field
    Reason reason = Reason.ACCEPTED;
    if (spread > MAX_SPREAD_M) {
      reason = Reason.SPREAD;
    } else if (Math.abs(now.xM + medianDx) > limitM || Math.abs(now.yM + medianDy) > limitM) {
      reason = Reason.OUT_OF_FIELD;
    } else if (!poseValid && !headingValid && headingGain == 0) {
      // Offsets turned by an untrusted heading put the robot anywhere. Once accepted, the pose is trusted and
      // the true fix is refused as a jump: one tag with the start heading 154 degrees out put the robot 2 m
      // away, on the far side of the Cell, for the whole run (5 Nov, 082826)
      reason = Reason.NO_HEADING;
    } else if (poseValid && Math.hypot(medianDx, medianDy) > MAX_JUMP_M) {
      reason = Reason.JUMP;
    }
    if (reason == Reason.JUMP) {
      boolean agrees = jumpAgreeFrames > 0
          && Math.hypot(medianDx - lastJumpDxM, medianDy - lastJumpDyM) <= POSE_OVERRIDE_AGREE_M;
      jumpAgreeFrames = agrees ? jumpAgreeFrames + 1 : 1;
      lastJumpDxM = medianDx;
      lastJumpDyM = medianDy;
      if (jumpAgreeFrames >= POSE_OVERRIDE_FRAMES) {
        reason = Reason.RELOCATED;
        jumpAgreeFrames = 0;
      }
    } else if (reason == Reason.ACCEPTED) {
      jumpAgreeFrames = 0;
    }
    boolean headingStillGood = reason == Reason.OUT_OF_FIELD || reason == Reason.JUMP; // Not about the heading
    if (reason != Reason.ACCEPTED && headingStillGood && headingGain > 0) {
      // The heading from tag pairs doesn't depend on where the robot is, so keep it
      Pose turned = new Pose(now.xM, now.yM, ShootOnTheMove.normalizeAngle(now.headingRad + headingGain * dHeading));
      VisionResult headingOnly = new VisionResult(Reason.HEADING_ONLY, firstReject, 0, 0, 0, turned, used,
          rejected, spread, redFlipped, blueFlipped, usedIds, usedDx, usedDy);
      headingOnly.dHeadingRad = dHeading;
      headingOnly.headingGain = headingGain;
      headingOnly.headingPairs = headingFixes.size();
      return headingOnly;
    }
    if (reason != Reason.ACCEPTED && reason != Reason.RELOCATED) {
      VisionResult rejectedResult = new VisionResult(reason, firstReject, 0, 0, 0, null, used, rejected,
          spread, redFlipped, blueFlipped, usedIds, usedDx, usedDy);
      rejectedResult.dHeadingRad = dHeading;
      rejectedResult.headingPairs = headingFixes.size();
      return rejectedResult;
    }

    double gain = poseValid && reason != Reason.RELOCATED ? CORRECTION_GAIN : 1.0;
    Pose newPose = new Pose(now.xM + gain * medianDx, now.yM + gain * medianDy,
        ShootOnTheMove.normalizeAngle(now.headingRad + headingGain * dHeading));
    VisionResult result = new VisionResult(reason, firstReject, medianDx, medianDy, gain, newPose, used,
        rejected, spread, redFlipped, blueFlipped, usedIds, usedDx, usedDy);
    result.dHeadingRad = dHeading;
    result.headingGain = headingGain;
    result.headingPairs = headingFixes.size();
    return result;
  }

  private static VisionResult fail(Reason reason, TagReject firstReject, int rejected,
      boolean redFlipped, boolean blueFlipped) {
    return new VisionResult(reason, firstReject, 0, 0, 0, null, 0, rejected, Double.NaN,
        redFlipped, blueFlipped, new int[0], new double[0], new double[0]);
  }

  /**
   * Counts one frame's votes for one Hive. No votes, or a tie, changes nothing. Returns true when
   * the up Cell flipped.
   */
  private boolean applyVotes(int hive, int[] votes, Alliance alliance) {
    int audience = votes[Cell.AUDIENCE.ordinal()];
    int scoring = votes[Cell.SCORING.ordinal()];
    if (audience == scoring) {
      return false;
    }
    Cell winner = audience > scoring ? Cell.AUDIENCE : Cell.SCORING;
    if (winner == upCell(alliance)) {
      flipVotes[hive] = 0;
      upCellConfirmed[hive] = true;
      return false;
    }
    if (++flipVotes[hive] < Math.max(1, TIP_CONFIRM_FRAMES)) {
      return false;
    }
    flipVotes[hive] = 0;
    upCellConfirmed[hive] = true;
    if (alliance == Alliance.RED) {
      upCellRed = winner;
    } else {
      upCellBlue = winner;
    }
    return true;
  }

  /** The middle value of a non-empty array. Leaves the input alone. */
  private static double median(double[] values) {
    double[] sorted = Arrays.copyOf(values, values.length);
    Arrays.sort(sorted);
    int n = sorted.length;
    return n % 2 == 1 ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
  }
}
