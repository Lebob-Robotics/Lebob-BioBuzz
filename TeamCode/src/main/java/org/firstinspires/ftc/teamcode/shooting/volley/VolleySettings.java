package org.firstinspires.ftc.teamcode.shooting.volley;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyAutoTuner.Settings;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.opmodes.tuning.VolleyTuning;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * The volley settings Lebot Teleop and the scoring auto shoot with, from Volley Tuning's CSV. Only confirmed spots
 * count: an unconfirmed estimate can come from an old feed (spot 0's is 12 Nov's 0.6 feeder). A shot near a confirmed
 * spot uses that spot's; anywhere else the confirmed spot nearest its distance to the Cell, a guess: the trims are
 * launch-angle offsets on the shot table, so the table still does the distance and angle, and what is left mostly
 * follows the distance. With no CSV the ShootOnTheMoveConstants VOLLEY_* values apply, set from the same data.
 */
public final class VolleySettings {
  static final double NEAR_SPOT_M = 0.5;

  // The grid and the park (VolleySpotDrive.PARK); sized by the grid alone the park's tuning was never loaded
  private final Settings[] spots = new Settings[VolleySpotDrive.SPOTS];
  private final boolean[] confirmed = new boolean[VolleySpotDrive.SPOTS];
  private final double[] rates = new double[VolleySpotDrive.SPOTS];

  /** Volley Tuning's results on the Hub; empty (constants) if the file is missing. */
  public static VolleySettings loadTuned() {
    return load(VolleyTuning.CSV_PATH);
  }

  /** Reads the CSV at {@code path}; empty (constants) if it is missing or unreadable. */
  public static VolleySettings load(String path) {
    List<String> lines = new ArrayList<>();
    File file = new File(path);
    if (file.exists()) {
      try (BufferedReader in = new BufferedReader(new FileReader(file))) {
        for (String line = in.readLine(); line != null; line = in.readLine()) { lines.add(line); }
      } catch (IOException e) {
        RobotLog.ee("Volley", e, "Can't load volley calibration; using the constants");
        lines.clear();
      }
    }
    return fromLines(lines);
  }

  static VolleySettings fromLines(List<String> lines) {
    VolleySettings v = new VolleySettings();
    if (lines.isEmpty()) { return v; }
    List<List<String>> bySpot = VolleyTuning.rowsBySpot(lines, v.spots.length);
    for (int i = 0; i < v.spots.length; i++) {
      List<String> rows = bySpot.get(i);
      if (rows.size() < 2) { continue; }
      VolleyAutoTuner tuner = new VolleyAutoTuner();
      tuner.replay(rows);
      if (!tuner.done()) { continue; }
      v.spots[i] = tuner.estimate();
      v.confirmed[i] = v.spots[i] != null;
      v.rates[i] = tuner.estimateRate();
    }
    return v;
  }

  /**
   * The confirmed spot whose settings apply at {@code pose}: one within NEAR_SPOT_M, else the one whose distance to
   * the Cell is nearest the robot's. -1 for the constants.
   */
  public int spotFor(FieldPose pose, double cellX, double cellY) {
    return spotFor(pose, cellX, cellY, false);
  }

  /** As above, among the backward spots for a backward volley and the hooded ones otherwise. */
  public int spotFor(FieldPose pose, double cellX, double cellY, boolean backward) {
    boolean[] known = new boolean[spots.length];
    for (int i = 0; i < spots.length; i++) {
      known[i] = spots[i] != null && VolleySpotDrive.isBackward(i) == backward;
    }
    if (pose == null) { return -1; }
    int near = nearestSpot(pose, cellX, cellY, known);
    return near >= 0 ? near : nearestRange(pose, cellX, cellY, known);
  }

  /** The known spot whose distance to the Cell is nearest the robot's; -1 if none is known. */
  static int nearestRange(FieldPose pose, double cellX, double cellY, boolean[] known) {
    double range = Math.hypot(pose.xM - cellX, pose.yM - cellY);
    int best = -1;
    double off = Double.POSITIVE_INFINITY;
    for (int i = 0; i < known.length; i++) {
      if (!known[i]) { continue; }
      double[] p = VolleySpotDrive.point(i, cellX, cellY);
      double d = Math.abs(Math.hypot(p[0] - cellX, p[1] - cellY) - range);
      if (d < off) { best = i; off = d; }
    }
    return best;
  }

  public Settings get(int spot) { return spot >= 0 ? spots[spot] : null; }

  boolean isConfirmed(int spot) { return spot >= 0 && confirmed[spot]; }

  public boolean isNear(FieldPose pose, double cellX, double cellY, int spot) {
    if (spot < 0 || pose == null) { return false; }
    double[] p = VolleySpotDrive.point(spot, cellX, cellY);
    return Math.hypot(p[0] - pose.xM, p[1] - pose.yM) <= NEAR_SPOT_M;
  }

  public static int nearestSpot(FieldPose pose, double cellX, double cellY, boolean[] known) {
    int best = -1;
    double distance = NEAR_SPOT_M;
    for (int i = 0; i < known.length; i++) {
      if (!known[i]) { continue; }
      double[] p = VolleySpotDrive.point(i, cellX, cellY);
      double d = Math.hypot(p[0] - pose.xM, p[1] - pose.yM);
      if (d <= distance) { best = i; distance = d; }
    }
    return best;
  }

  /** Puts these settings (null: the constants) on the robot's volley. */
  public static void apply(Robot robot, Settings s) {
    apply(robot, s, false);
  }

  /** As above; {@code backward} for a backward spot's settings, whose trims are wheel speed. */
  public static void apply(Robot robot, Settings s, boolean backward) {
    robot.setVolleyTrimsDeg(s == null ? Double.NaN : s.firstTrim, s == null ? Double.NaN : s.laterTrim, backward);
    robot.setVolleyHeadingTrimDeg(s == null ? Double.NaN : s.headingTrim);
    robot.setVolleyFeedPower(s == null ? Double.NaN : s.feeder);
    robot.setVolleyIndexerPower(s == null ? Double.NaN : s.feeder * s.indexerScale);
    robot.setVolleyBoostRpm(s == null ? Double.NaN : s.boostRpm);
    robot.setVolleyFastRecovery(s == null ? null : s.fastRecovery);
  }
}
