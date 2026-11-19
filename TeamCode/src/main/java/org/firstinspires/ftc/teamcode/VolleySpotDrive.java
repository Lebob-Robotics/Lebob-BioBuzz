package org.firstinspires.ftc.teamcode;

import java.util.List;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShotTuningSettings;

/** Volley Tuning uses Shot Tuning's spots, safe routes and persistent field-point hold. */
final class VolleySpotDrive {
  enum Stage { IDLE, ACQUIRING, MOVING, SETTLING, HOLDING }
  static final double SETTLE_SECONDS = 1.0;
  static final double MAX_BLIND_HOLD_ERROR_M = 0.20;
  private Stage stage = Stage.IDLE;
  private int spot = -1;
  private double searchSince, searchStep, turn;
  private double[] target;
  private List<double[]> route;
  private int leg;
  private double legX, legY, progressX, progressY, progressS, lastFixS, deadlineS;
  private double settledSince = Double.NaN;
  private int fixes;
  private String status = "Circle: move to the first test spot";
  private double[] drive = new double[] {0, 0};

  static double[] point(int index, double cellX, double cellY) {
    return new double[] {FieldRoute.GRID[index][0] * (cellX < 0 ? 1 : -1),
        FieldRoute.GRID[index][1] * (cellY < 0 ? 1 : -1)};
  }

  static int nextSpot(int current, boolean[] done) {
    for (int n = 1; n <= done.length; n++) {
      int i = (current + n) % done.length;
      if (!done[i]) { return i; }
    }
    return -1;
  }

  boolean start(int index, double cellX, double cellY, double now, Pose pose, boolean valid, int fixCount) {
    if (!valid || !Double.isFinite(cellX) || !Double.isFinite(cellY)) {
      stop("Finding Hive tags for spot " + (index + 1) + "; Circle cancels");
      spot = index; searchSince = searchStep = now; stage = Stage.ACQUIRING;
      return true;
    }
    double[] end = point(index, cellX, cellY);
    List<double[]> path = FieldRoute.route(pose.xM, pose.yM, end[0], end[1]);
    double x = pose.xM, y = pose.yM, length = 0;
    for (double[] p : path) {
      if (!FieldRoute.pathClearEntering(x, y, p[0], p[1])) {
        stop("No safe route to this spot");
        return false;
      }
      length += Math.hypot(p[0] - x, p[1] - y);
      x = p[0]; y = p[1];
    }
    spot = index; target = end; route = path; leg = 0;
    legX = progressX = pose.xM; legY = progressY = pose.yM;
    progressS = lastFixS = now; fixes = fixCount;
    deadlineS = now + FieldRoute.GOTO_TIMEOUT_S + length / 0.4;
    settledSince = Double.NaN;
    stage = Stage.MOVING;
    status = "Moving to spot " + (spot + 1) + "; Circle cancels";
    return true;
  }

  /** Rotate only until vision supplies a trusted pose and Cell, then fulfill the queued Circle. */
  void acquire(double now, Pose pose, boolean valid, int fixCount,
      double cellX, double cellY, double tagsAgo) {
    turn = 0;
    if (stage != Stage.ACQUIRING) { return; }
    if (now - searchSince >= ShotTuningSettings.FIND_TAGS_TIMEOUT_S) {
      stop("No reliable Hive pose after 30s; Circle retries");
      return;
    }
    if (valid && Double.isFinite(cellX) && Double.isFinite(cellY)) {
      start(spot, cellX, cellY, now, pose, true, fixCount);
      return;
    }
    boolean seen = tagsAgo < 0.3;
    double pause = seen ? ShotTuningSettings.FIND_TAGS_SEEN_PAUSE_S : ShotTuningSettings.FIND_TAGS_PAUSE_S;
    double elapsed = now - searchStep;
    if (elapsed >= pause + ShotTuningSettings.FIND_TAGS_STEP_S) {
      searchStep = now; elapsed = 0;
    }
    turn = elapsed < pause ? 0 : ShotTuningSettings.FIND_TAGS_TURN;
    status = "Finding Hive tags for spot " + (spot + 1)
        + (seen ? ": waiting for a reliable pose; Circle cancels" : "; Circle cancels");
  }

  void update(double now, Pose pose, double[] velocity, boolean valid, int fixCount) {
    if (stage == Stage.IDLE || stage == Stage.ACQUIRING) { drive = new double[] {0, 0}; return; }
    if (fixCount != fixes) { fixes = fixCount; lastFixS = now; }
    if (!valid) { stop("Stopped: field pose invalid; Circle retries"); return; }
    if (!FieldRoute.headedSafelyEntering(pose, velocity)) {
      stop("Stopped: unsafe predicted stopping point; Circle retries"); return;
    }
    // A parked robot keeps its trusted odometry pose when balls/hands hide the tags. Keep the
    // loading hold and RB trial available, with only small position corrections while blind.
    boolean blind = now - lastFixS > FieldRoute.GOTO_BLIND_S;
    if (blind && (stage == Stage.MOVING
        || Math.hypot(target[0] - pose.xM, target[1] - pose.yM) > MAX_BLIND_HOLD_ERROR_M)) {
      stop(stage == Stage.MOVING ? "Stopped: tag fixes lost on route; Circle retries"
          : "Stopped: displaced too far from the spot without tags; Circle retries");
      return;
    }
    if (stage == Stage.MOVING) {
      if (now > deadlineS) { stop("Route timed out; Circle retries"); return; }
      if (Math.hypot(pose.xM - progressX, pose.yM - progressY) >= 0.05) {
        progressX = pose.xM; progressY = pose.yM; progressS = now;
      }
      double[] p = route.get(leg);
      boolean last = leg == route.size() - 1;
      double away = Math.hypot(p[0] - pose.xM, p[1] - pose.yM);
      if (last && away <= FieldRoute.GOTO_TOLERANCE_M) {
        stage = Stage.SETTLING;
      } else {
        if (!last && away <= FieldRoute.ROUTE_WAYPOINT_TOLERANCE_M) {
          leg++; legX = pose.xM; legY = pose.yM; p = route.get(leg);
        }
        drive = FieldRoute.goToStick(pose, p[0], p[1], legX, legY,
            FieldRoute.GOTO_STUCK_BOOST_PER_S * (now - progressS));
        return;
      }
    }
    // A null drive lets Robot's persistent holdPoint return it after a loading nudge.
    drive = null;
    boolean settled = Math.hypot(target[0] - pose.xM, target[1] - pose.yM) <= FieldRoute.GOTO_TOLERANCE_M
        && Math.hypot(velocity[0], velocity[1]) < 0.08;
    if (!settled) { settledSince = Double.NaN; stage = Stage.SETTLING; }
    else if (Double.isNaN(settledSince)) { settledSince = now; }
    else if (now - settledSince >= SETTLE_SECONDS) { stage = Stage.HOLDING; }
    status = stage == Stage.HOLDING ? "Holding spot " + (spot + 1) + "; load four with LT, hold RB to fire"
        : "Settling at spot " + (spot + 1);
  }

  void stop(String why) { stage = Stage.IDLE; turn = 0; drive = new double[] {0, 0}; status = why; }
  Stage stage() { return stage; }
  int spot() { return spot; }
  boolean ready() { return stage == Stage.HOLDING; }
  boolean shouldAim() { return stage != Stage.IDLE; }
  boolean active() { return stage != Stage.IDLE && stage != Stage.ACQUIRING; }
  double turn() { return turn; }
  double[] drive() { return drive == null ? null : drive.clone(); }
  double[] holdPoint() { return active() && stage != Stage.MOVING ? target.clone() : null; }
  String status() { return status; }
}
