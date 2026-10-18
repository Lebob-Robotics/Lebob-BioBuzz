package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning values for shoot on the move, editable live from Panels. Metres, seconds and radians unless
 * the name says otherwise.
 *
 * <p>Every value is an estimate until the robot is built and measured. The hood angle and RPM for
 * each shot are not here: they are in the generated {@link ShotTable}, made from
 * tools/shots/inputs.json.
 */
@Configurable
public final class ShootOnTheMoveConstants {
  private ShootOnTheMoveConstants() {}

  public static double FEED_DELAY_S = 0.10; // Fire command to ball leaving the shooter. Sean's estimate

  // ---- Shooter on the robot (Sean's CAD) -------------------------------
  public static double SHOOTER_DIRECTION_RAD = Math.PI; // Relative to robot forward. PI fires out of the back
  public static double SHOOTER_FORWARD_M = 0.0612; // Flywheel position from the robot centre
  public static double SHOOTER_LEFT_M = -0.0179;

  // ---- Cell (Sean's field CAD) -----------------------------------------
  /**
   * How much nearer the Cell's near lip is than its tag cluster centre, seen from the robot. The
   * opening centre is about 0.157 m out from the cluster, and the near lip a further 0.089 m.
   */
  public static double CLUSTER_TO_NEAR_LIP_M = 0.25;
  public static double MIN_UP_CELL_HEIGHT_M = 1.08; // Halfway between the up (1.26 m) and down (0.90 m) Cells' clusters
  public static double CELL_HALF_WIDTH_M = 0.254; // Opening is 20 in wide
  public static double BALL_RADIUS_M = 0.0356; // Pollen, 2.8 in across
  public static double SIDE_MARGIN_M = 0.02; // Kept clear of each side of the opening

  // ---- Remembering the Cell --------------------------------------------
  public static double CELL_EXPIRY_S = 3.0; // Forget the Cell this long after the last sighting
  public static double CELL_SMOOTHING = 0.3; // Share of the way the stored Cell moves toward each sighting. 1 is no smoothing
  public static final int POSE_HISTORY_SIZE = 100; // About two seconds of loops

  // ---- Cameras (Sean's CAD: both face backward, like the shooter) ------
  public static final CameraMount LEFT_CAMERA = new CameraMount(
      -0.07405, 0.04313, 0.16972, Math.PI - Math.toRadians(35), Math.toRadians(48.907));
  public static final CameraMount RIGHT_CAMERA = new CameraMount(
      -0.07442, -0.04318, 0.17040, -(Math.PI - Math.toRadians(35)), Math.toRadians(48.907));

  // ---- Shot checks -----------------------------------------------------
  public static double MAX_TURNING_DEG = 30; // Above this, the ball's sideways error grows too fast
  /**
   * Neighbouring table cells are only blended when their hood angles are this close. Further apart
   * they are different kinds of shot, and a blend may miss, so the nearest cell is used.
   */
  public static double MAX_HOOD_DIFFERENCE_TO_BLEND_DEG = 4;

  // ---- Fire check (match the tolerances in tools/shots/inputs.json) ----
  public static double RPM_TOLERANCE = 50; // Per flywheel
  public static double HOOD_TOLERANCE_DEG = 1.0;
  public static double MAX_HEADING_TOLERANCE_DEG = 5; // Otherwise it grows large at short range

  // ---- Driver ----------------------------------------------------------
  public static double RPM_TRIM_STEP = 50; // Per D-pad press while aiming
  public static double AIM_START_RPM = 2500; // Held while aiming until the first valid shot. Middle of the table
}
