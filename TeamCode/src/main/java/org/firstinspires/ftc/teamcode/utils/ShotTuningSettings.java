package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/** Shot Tuning's search and moving-pass settings, editable live from Panels. */
@Configurable
public final class ShotTuningSettings {
  private ShotTuningSettings() {}

  // ---- Search at each spot: each marked ball moves the hood (or the RPM, where the table's hood is on a stop)
  public static boolean AUTO_SEARCH = true;
  public static double SEARCH_HOOD_STEP_DEG = 2.0; // Before the range is bracketed
  public static double SEARCH_HOOD_RESOLUTION_DEG = 0.5; // Each edge is found to within this
  public static double SEARCH_RPM_STEP = 100;
  public static double SEARCH_RPM_RESOLUTION = 25;
  public static int CONFIRM_BALLS = 3; // At the middle of the range, once both edges are known
  public static double SEARCH_MOVED_M = 0.15; // Further than this from where the search began, marks no longer feed it
  // Past vertical (90) the backward shot leans back over the robot; 90 scored from 0.4 m
  public static double BACKWARD_SEARCH_MIN_DEG = 88;

  // ---- Backward spots: the cameras face away from the Cell, so the robot turns them to the Hive to move and to
  // check the spot, then turns around to shoot
  // From 0.4 m (G0) the cameras see no Hive tags at all, so a fix just before arriving counts too
  public static int SPOT_TAG_FIXES = 2; // Tag fixes after turning before it drives
  public static double LOOK_FIX_WAIT_S = 2; // Drives anyway after this; the drive stops if it sees no tags
  public static double SPOT_FIX_M = 0.04; // A fix at the spot puts it this close before it turns around
  public static double SPOT_RECENT_FIX_S = 1.0; // Or a fix this soon before arriving
  public static double LOOK_DONE_DEG = 5;
  public static double LOOK_TIMEOUT_S = 4; // To turn the cameras to the Hive
  public static double SETTLE_MAX_S = 4; // Turns around after this even without a fix
  // While the Cell or pose isn't known (as after a restart facing away), it turns in steps to find the tags,
  // stopping between them: turning, the tags are rejected as too fast
  public static double FIND_TAGS_TURN = 0.3;
  public static double FIND_TAGS_STEP_S = 0.35;
  public static double FIND_TAGS_PAUSE_S = 0.8;
  public static double FIND_TAGS_SEEN_PAUSE_S = 2.5; // Stops this long while it sees tags, for a fix
  public static double FIND_TAGS_TIMEOUT_S = 30;

  // ---- Aim trim from left and right misses
  public static boolean AUTO_TRIM = true;
  public static double TRIM_STEP_DEG = 1.0;
  public static double TRIM_MIN_STEP_DEG = 0.25;

  // ---- Moving passes (Start): drive through the spot and fire one ball as it passes
  public static double PASS_HALF_LENGTH_M = 0.35; // Start and end this far either side of the spot
  public static double PASS_STICK_HOODED = 0.5;
  // Backward shots are only taken up to ShootOnTheMoveConstants.BACKWARD_MAX_SPEED_MPS
  public static double PASS_STICK_BACKWARD = 0.25;
  public static double PASS_FIRE_LEAD_M = 0.10; // Asks for the ball this far before the spot
  public static double PASS_READY_S = 1.0; // Lines up standing at the start first
}
