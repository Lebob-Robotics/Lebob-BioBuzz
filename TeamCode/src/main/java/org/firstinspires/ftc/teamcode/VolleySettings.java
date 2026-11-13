package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner.Settings;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Lebot Teleop's RB volley settings, from Volley Tuning's CSV. Each tuned spot gives its confirmed setting, or until
 * then its best estimate ({@link VolleyAutoTuner#estimate}). A shot near a tuned spot uses that spot's; anywhere
 * else uses the best tuned spot's, a guess: the trims are launch-angle offsets on the shot table, so the table
 * still does the distance and angle and the offsets carry over until those spots are tuned too. With no CSV the
 * ShootOnTheMoveConstants VOLLEY_* values apply, which are set from the same data.
 */
final class VolleySettings {
  static final double NEAR_SPOT_M = 0.5;

  private final Settings[] spots = new Settings[FieldRoute.GRID.length];
  private final boolean[] confirmed = new boolean[FieldRoute.GRID.length];
  private final double[] rates = new double[FieldRoute.GRID.length];
  private int fallback = -1; // The spot used away from every tuned spot

  /** Reads the CSV at {@code path}; empty (constants) if it is missing or unreadable. */
  static VolleySettings load(String path) {
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
    for (int i = 0; i < v.spots.length; i++) {
      List<String> rows = VolleyTuning.rowsForSpot(lines, i);
      if (rows.size() < 2) { continue; }
      VolleyAutoTuner tuner = new VolleyAutoTuner();
      tuner.replay(rows);
      v.spots[i] = tuner.estimate();
      v.confirmed[i] = tuner.done();
      v.rates[i] = tuner.estimateRate();
    }
    // Away from tuned spots: a confirmed spot first, then the one whose estimate puts the most balls in
    for (int i = 0; i < v.spots.length; i++) {
      if (v.spots[i] == null) { continue; }
      if (v.fallback < 0 || v.confirmed[i] && !v.confirmed[v.fallback]
          || v.confirmed[i] == v.confirmed[v.fallback] && v.rates[i] > v.rates[v.fallback]) {
        v.fallback = i;
      }
    }
    return v;
  }

  /** The tuned spot whose settings apply at {@code pose}; -1 for the constants. */
  int spotFor(Pose pose, double cellX, double cellY) {
    boolean[] known = new boolean[spots.length];
    for (int i = 0; i < spots.length; i++) { known[i] = spots[i] != null; }
    int near = pose == null ? -1 : nearestSpot(pose, cellX, cellY, known);
    return near >= 0 ? near : fallback;
  }

  Settings get(int spot) { return spot >= 0 ? spots[spot] : null; }

  boolean isConfirmed(int spot) { return spot >= 0 && confirmed[spot]; }

  boolean isNear(Pose pose, double cellX, double cellY, int spot) {
    if (spot < 0 || pose == null) { return false; }
    double[] p = VolleySpotDrive.point(spot, cellX, cellY);
    return Math.hypot(p[0] - pose.xM, p[1] - pose.yM) <= NEAR_SPOT_M;
  }

  static int nearestSpot(Pose pose, double cellX, double cellY, boolean[] known) {
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
  static void apply(Robot robot, Settings s) {
    robot.setVolleyTrimsDeg(s == null ? Double.NaN : s.firstTrim, s == null ? Double.NaN : s.laterTrim);
    robot.setVolleyHeadingTrimDeg(s == null ? Double.NaN : s.headingTrim);
    robot.setVolleyFeedPower(s == null ? Double.NaN : s.feeder);
    robot.setVolleyIndexerPower(s == null ? Double.NaN : s.feeder * s.indexerScale);
    robot.setVolleyBoostRpm(s == null ? Double.NaN : s.boostRpm);
    robot.setVolleyFastRecovery(s == null ? null : s.fastRecovery);
  }
}
