package org.firstinspires.ftc.teamcode;

import java.util.List;
import org.firstinspires.ftc.teamcode.auto.AutoField;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShotTuningSettings;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;

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

  /**
   * The spot after the grid: where the Scoring Auto parks (AutoField.parkSpot), against our wall in the loading
   * zone. The drive stops {@link #PARK_APPROACH_M} from the middle, inside the route's field limit, then creeps
   * toward the wall until the wall stops it, and holds there pushed against it as the auto does.
   */
  static final int PARK = FieldRoute.GRID.length;
  static final int SPOTS = PARK + 1;
  static final double PARK_APPROACH_M = 1.30;
  static final double PARK_CREEP_STICK = 0.25;
  static final double PARK_PUSH_STICK = 0.15;
  static final double PARK_X_GAIN = 2.0; // Stick per metre along the wall, held to the park's x
  static final double PARK_STOPPED_MPS = 0.05; // Toward the wall, for PARK_STOPPED_S: the wall has it
  static final double PARK_STOPPED_S = 0.25;
  static final double PARK_CREEP_MIN_S = 0.4;
  static final double PARK_CREEP_MAX_S = 4.0;
  /** Beyond the wall's reach by this much the pose is wrong: stop. */
  static final double PARK_POSE_SLACK_M = 0.10;

  static double[] point(int index, double cellX, double cellY) {
    double[] p = index == PARK ? parkPoint() : FieldRoute.GRID[index];
    return new double[] {p[0] * (cellX < 0 ? 1 : -1), p[1] * (cellY < 0 ? 1 : -1)};
  }

  /** The park for the Cell at -x, -y, as the grid's spots are: red's, back to its -x Cell. */
  static double[] parkPoint() {
    for (VisionCorrector.Cell c : VisionCorrector.Cell.values()) {
      if (VisionCorrector.upCellAimPoint(Alliance.RED, c)[0] < 0) {
        double[] p = AutoField.parkSpot(Alliance.RED, c);
        return new double[] {p[0], p[1]};
      }
    }
    throw new IllegalStateException("No red Cell at -x");
  }

  static String name(int index) {
    return index == PARK ? "PARK" : Integer.toString(index + 1);
  }

  private boolean creeping;
  private double creepSince, stoppedSince;

  static int nextSpot(int current, boolean[] done) {
    for (int n = 1; n <= done.length; n++) {
      int i = (current + n) % done.length;
      if (!done[i]) { return i; }
    }
    return -1;
  }

  boolean start(int index, double cellX, double cellY, double now, Pose pose, boolean valid, int fixCount) {
    if (!valid || !Double.isFinite(cellX) || !Double.isFinite(cellY)) {
      stop("Finding Hive tags for spot " + name(index) + "; Circle cancels");
      spot = index; searchSince = searchStep = now; stage = Stage.ACQUIRING;
      return true;
    }
    double[] end = point(index, cellX, cellY);
    double[] stop = index == PARK ? new double[] {end[0], Math.signum(end[1]) * PARK_APPROACH_M} : end;
    List<double[]> path = FieldRoute.route(pose.xM, pose.yM, stop[0], stop[1]);
    double x = pose.xM, y = pose.yM, length = 0;
    for (double[] p : path) {
      if (!FieldRoute.pathClearEntering(x, y, p[0], p[1])) {
        stop("No safe route to this spot");
        return false;
      }
      length += Math.hypot(p[0] - x, p[1] - y);
      x = p[0]; y = p[1];
    }
    spot = index; target = end; route = path; leg = 0; creeping = false;
    legX = progressX = pose.xM; legY = progressY = pose.yM;
    progressS = lastFixS = now; fixes = fixCount;
    deadlineS = now + FieldRoute.GOTO_TIMEOUT_S + length / 0.4;
    settledSince = Double.NaN;
    stage = Stage.MOVING;
    status = "Moving to spot " + name(spot) + "; Circle cancels";
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
    status = "Finding Hive tags for spot " + name(spot)
        + (seen ? ": waiting for a reliable pose; Circle cancels" : "; Circle cancels");
  }

  void update(double now, Pose pose, double[] velocity, boolean valid, int fixCount) {
    if (stage == Stage.IDLE || stage == Stage.ACQUIRING) { drive = new double[] {0, 0}; return; }
    if (fixCount != fixes) { fixes = fixCount; lastFixS = now; }
    if (!valid) { stop("Stopped: field pose invalid; Circle retries"); return; }
    if (spot == PARK && (creeping || stage == Stage.HOLDING)) {
      updatePark(now, pose, velocity);
      return;
    }
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
      if (last && away <= FieldRoute.GOTO_TOLERANCE_M && spot == PARK) {
        creeping = true; creepSince = now; stoppedSince = Double.NaN;
        updatePark(now, pose, velocity);
        return;
      } else if (last && away <= FieldRoute.GOTO_TOLERANCE_M) {
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
    status = stage == Stage.HOLDING ? "Holding spot " + name(spot) + "; load four with LT, hold RB to fire"
        : "Settling at spot " + name(spot);
  }

  /**
   * The park's last bit: creeps toward our wall (held to the park's x) until the wall stops it, then holds there
   * pushed against it. The wall is the stop, so the route's stopping check is replaced by a reach check on the pose.
   */
  private void updatePark(double now, Pose pose, double[] velocity) {
    double wall = Math.signum(target[1]);
    if (Math.abs(pose.yM) > FieldRoute.MAX_REACH_M + PARK_POSE_SLACK_M
        || Math.abs(pose.xM - target[0]) > MAX_BLIND_HOLD_ERROR_M) {
      stop("Stopped: park pose off the wall's reach; Circle retries");
      return;
    }
    double alongX = Math.max(-GOTO_PARK_X_CAP, Math.min(GOTO_PARK_X_CAP, PARK_X_GAIN * (target[0] - pose.xM)));
    if (creeping) {
      double towardWall = velocity[1] * wall;
      if (now - creepSince >= PARK_CREEP_MIN_S && towardWall < PARK_STOPPED_MPS) {
        if (Double.isNaN(stoppedSince)) { stoppedSince = now; }
      } else {
        stoppedSince = Double.NaN;
      }
      if (!Double.isNaN(stoppedSince) && now - stoppedSince >= PARK_STOPPED_S) {
        creeping = false;
        target = new double[] {target[0], pose.yM}; // Where the wall has it
        stage = Stage.HOLDING;
      } else if (now - creepSince > PARK_CREEP_MAX_S) {
        stop("Stopped: never reached our wall; Circle retries");
        return;
      } else {
        drive = new double[] {alongX, wall * PARK_CREEP_STICK};
        status = "Creeping onto our wall at the park";
        return;
      }
    }
    drive = new double[] {alongX, wall * PARK_PUSH_STICK};
    status = "Holding the PARK against our wall; load four with LT, hold RB to fire";
  }

  private static final double GOTO_PARK_X_CAP = 0.3;

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
