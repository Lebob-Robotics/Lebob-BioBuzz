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
 * <p>The odometry heading must be in the field frame, because the tags have field positions. Only x
 * and y are corrected: the heading always comes from the Pinpoint. Robot frame is x forward, y left,
 * z up. Keep one corrector for the OpMode, since it holds the tip votes.
 */
public final class VisionCorrector {
  /** One end of a Hive. Exactly one of the two Cells is up at a time. */
  public enum Cell { AUDIENCE, SCORING }

  private Cell upCellRed = Cell.AUDIENCE; // Starts red AUDIENCE, blue SCORING. Vision flips them
  private Cell upCellBlue = Cell.SCORING;
  private final int[] flipVotes = new int[2]; // Frames in a row voting against the current up Cell: red, blue

  /** The up Cell of an alliance's Hive. */
  public Cell upCell(Alliance alliance) {
    return alliance == Alliance.RED ? upCellRed : upCellBlue;
  }

  /** Clears partial tip votes. */
  public void reset() {
    flipVotes[0] = 0;
    flipVotes[1] = 0;
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
    List<Double> dxs = new ArrayList<>();
    List<Double> dys = new ArrayList<>();
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

      // Robot position is the tag position less the tag's offset from the robot, turned into the field frame
      double cosHeading = Math.cos(atFrame.headingRad);
      double sinHeading = Math.sin(atFrame.headingRad);
      double offsetX = inRobot[0] * cosHeading - inRobot[1] * sinHeading;
      double offsetY = inRobot[0] * sinHeading + inRobot[1] * cosHeading;
      dxs.add(tagField[0] - offsetX - atFrame.xM);
      dys.add(tagField[1] - offsetY - atFrame.yM);
      ids.add(tag.id);
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
    double spread = 0;
    for (int i = 0; i < used; i++) {
      spread = Math.max(spread, Math.hypot(usedDx[i] - medianDx, usedDy[i] - medianDy));
    }

    double limitM = FIELD_HALF_SIZE_M - ROBOT_HALF_SIZE_M; // Where vision says the robot is now must be on the field
    Reason reason = Reason.ACCEPTED;
    if (spread > MAX_SPREAD_M) {
      reason = Reason.SPREAD;
    } else if (Math.abs(now.xM + medianDx) > limitM || Math.abs(now.yM + medianDy) > limitM) {
      reason = Reason.OUT_OF_FIELD;
    } else if (poseValid && Math.hypot(medianDx, medianDy) > MAX_JUMP_M) {
      reason = Reason.JUMP;
    }
    if (reason != Reason.ACCEPTED) {
      return new VisionResult(reason, firstReject, 0, 0, 0, null, used, rejected, spread,
          redFlipped, blueFlipped, usedIds, usedDx, usedDy);
    }

    double gain = poseValid ? CORRECTION_GAIN : 1.0;
    Pose newPose = new Pose(now.xM + gain * medianDx, now.yM + gain * medianDy, now.headingRad);
    return new VisionResult(reason, firstReject, medianDx, medianDy, gain, newPose, used, rejected,
        spread, redFlipped, blueFlipped, usedIds, usedDx, usedDy);
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
      return false;
    }
    if (++flipVotes[hive] < Math.max(1, TIP_CONFIRM_FRAMES)) {
      return false;
    }
    flipVotes[hive] = 0;
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
