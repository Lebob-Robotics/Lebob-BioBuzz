package org.firstinspires.ftc.teamcode.shooting.volley;

import java.util.List;
import org.firstinspires.ftc.teamcode.auto.AutoField;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.tuning.ShotTuningSettings;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldRoute;
import org.firstinspires.ftc.teamcode.field.RouteDriver;

/** Volley Tuning uses Shot Tuning's spots, safe routes and persistent field-point hold. */
public final class VolleySpotDrive {
  public enum Stage { IDLE, ACQUIRING, MOVING, SETTLING, HOLDING }
  static final double SETTLE_SECONDS = 1.0;
  static final double MAX_BLIND_HOLD_ERROR_M = 0.20;
  private Stage stage = Stage.IDLE;
  private int spot = -1;
  private double searchSince, searchStep, turn;
  private double[] target;
  private final RouteDriver route = new RouteDriver();
  private double deadlineS;
  private double settledSince = Double.NaN;
  private String status = "Circle: move to the first test spot";
  private double[] drive = new double[] {0, 0};

  /**
   * The spot after the grid: where the Scoring Auto parks (AutoField.parkSpot), against our wall in the loading
   * zone. The drive stops {@link #PARK_APPROACH_M} from the middle, inside the route's field limit, then creeps
   * toward the wall until the wall stops it, and holds there pushed against it as the auto does.
   */
  static final int PARK = FieldRoute.GRID.length;
  /**
   * Volley-only spots after the park, numbered on from it so saved spot numbers keep their meaning. Where TeleOp's
   * hooded volleys left from 0.3-0.6 m off every tuned spot: along the side wall 30-39 degrees off the Cell, and
   * level with the Cell just outside the backward wedge.
   */
  static final double[][] EXTRA = {
      {-1.30, -0.95}, // 11: 1.11 m from the aim point, 34 degrees off
      {-1.22, 0.20},  // 12: 0.94 m, 34 the other way
  };
  /**
   * Backward volley spots, after the extra ones: grid spots 1-4, straight out from the Cell, where TeleOp shoots
   * backward (BACKWARD_ONLY_WEDGE_DEG). Shown as B1-B4.
   */
  static final int BACKWARD_FIRST = PARK + 1 + EXTRA.length;
  static final int BACKWARD_SPOTS = 4;
  public static final int SPOTS = BACKWARD_FIRST + BACKWARD_SPOTS;

  public static boolean isBackward(int index) { return index >= BACKWARD_FIRST; }
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

  public static double[] point(int index, double cellX, double cellY) {
    double[] p = index == PARK ? parkPoint() : isBackward(index) ? FieldRoute.GRID[index - BACKWARD_FIRST]
        : index > PARK ? EXTRA[index - PARK - 1] : FieldRoute.GRID[index];
    return new double[] {p[0] * (cellX < 0 ? 1 : -1), p[1] * (cellY < 0 ? 1 : -1)};
  }

  /** The park for the Cell at -x, -y, as the grid's spots are: red's, back to its -x Cell. */
  static double[] parkPoint() {
    for (Cell c : Cell.values()) {
      if (VisionCorrector.upCellAimPoint(Alliance.RED, c)[0] < 0) {
        double[] p = AutoField.parkSpot(Alliance.RED, c);
        return new double[] {p[0], p[1]};
      }
    }
    throw new IllegalStateException("No red Cell at -x");
  }

  public static String name(int index) {
    return index == PARK ? "PARK" : isBackward(index) ? "B" + (index - BACKWARD_FIRST + 1)
        : Integer.toString(index > PARK ? index : index + 1);
  }

  private boolean creeping;
  private double creepSince, stoppedSince;

  /**
   * The spots Volley Tuning visits, in order: the auto's two shots (the park, spot 4), then the side spots TeleOp
   * volleys from, with the extra spots beside their neighbours. Not spots 0-3: they are within BACKWARD_ONLY_WEDGE_DEG
   * of straight out from the Cell, where TeleOp now shoots backward, and hooded volleys there bounced.
   */
  static final int[] VOLLEY_ORDER = {PARK, 4, PARK + 1, PARK + 2, 5, 6, 7, 8, 9};
  /**
   * The backward spots, most used first: TeleOp's backward volleys left from the side wall near B3 and B4; B1 is
   * 0.40 m out.
   */
  static final int[] BACKWARD_ORDER = {BACKWARD_FIRST + 2, BACKWARD_FIRST + 3, BACKWARD_FIRST + 1, BACKWARD_FIRST};

  public static int[] order(boolean backward) { return backward ? BACKWARD_ORDER : VOLLEY_ORDER; }

  /** The next unconfirmed spot in {@link #VOLLEY_ORDER} after {@code current}, wrapping; -1 when all are done. */
  public static int nextVolleySpot(int current, boolean[] done) {
    return nextVolleySpot(current, done, VOLLEY_ORDER);
  }

  /** The next unconfirmed spot in {@code order} after {@code current}, wrapping; -1 when all are done. */
  public static int nextVolleySpot(int current, boolean[] done, int[] order) {
    int at = -1;
    for (int i = 0; i < order.length; i++) { if (order[i] == current) { at = i; } }
    for (int n = 1; n <= order.length; n++) {
      int spot = order[(at + n) % order.length];
      if (spot < done.length && !done[spot]) { return spot; }
    }
    return -1;
  }

  /** The first unconfirmed spot in {@code order}; its first spot when all are done. */
  public static int firstUntuned(boolean[] done, int[] order) {
    for (int spot : order) { if (spot < done.length && !done[spot]) { return spot; } }
    return order[0];
  }

  static int nextSpot(int current, boolean[] done) {
    for (int n = 1; n <= done.length; n++) {
      int i = (current + n) % done.length;
      if (!done[i]) { return i; }
    }
    return -1;
  }

  public boolean start(int index, double cellX, double cellY, double now, FieldPose pose, boolean valid, int fixCount) {
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
    spot = index; target = end; creeping = false;
    route.start(now, pose, path, fixCount);
    deadlineS = now + FieldRoute.GOTO_TIMEOUT_S + length / 0.4;
    settledSince = Double.NaN;
    stage = Stage.MOVING;
    status = "Moving to spot " + name(spot) + "; Circle cancels";
    return true;
  }

  /** Rotate only until vision supplies a trusted pose and Cell, then fulfill the queued Circle. */
  public void acquire(double now, FieldPose pose, boolean valid, int fixCount,
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

  public void update(double now, FieldPose pose, double[] velocity, boolean valid, int fixCount) {
    if (stage == Stage.IDLE || stage == Stage.ACQUIRING) { drive = new double[] {0, 0}; return; }
    route.track(now, pose, fixCount);
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
    boolean blind = route.blind(now);
    if (blind && (stage == Stage.MOVING
        || Math.hypot(target[0] - pose.xM, target[1] - pose.yM) > MAX_BLIND_HOLD_ERROR_M)) {
      stop(stage == Stage.MOVING ? "Stopped: tag fixes lost on route; Circle retries"
          : "Stopped: displaced too far from the spot without tags; Circle retries");
      return;
    }
    if (stage == Stage.MOVING) {
      if (now > deadlineS) { stop("Route timed out; Circle retries"); return; }
      boolean arrived = route.arrived(pose);
      if (arrived && spot == PARK) {
        creeping = true; creepSince = now; stoppedSince = Double.NaN;
        updatePark(now, pose, velocity);
        return;
      } else if (arrived) {
        stage = Stage.SETTLING;
      } else {
        drive = route.stick(now, pose);
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
  private void updatePark(double now, FieldPose pose, double[] velocity) {
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

  public void stop(String why) { stage = Stage.IDLE; turn = 0; drive = new double[] {0, 0}; status = why; }
  public Stage stage() { return stage; }
  int spot() { return spot; }
  public boolean ready() { return stage == Stage.HOLDING; }
  public boolean shouldAim() { return stage != Stage.IDLE; }
  boolean active() { return stage != Stage.IDLE && stage != Stage.ACQUIRING; }
  public double turn() { return turn; }
  public double[] drive() { return drive == null ? null : drive.clone(); }
  public double[] holdPoint() { return active() && stage != Stage.MOVING ? target.clone() : null; }
  public String status() { return status; }
}
