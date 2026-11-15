package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Where an aimed shot points: the Cell it aims at, how far ahead of the robot's motion it leads, the trims, and the
 * checks a shot must pass to exist. Editable live from Panels. Metres, seconds and radians unless the name says
 * otherwise. The hood angle and RPM of each shot are in the generated {@link ShotTable}, from tools/shots/inputs.json.
 */
@Configurable
public final class AimConstants {
  private AimConstants() {}

  // Fire command to ball leaving the shooter. The flywheels' RPM dips as late as 0.30-0.35 s after the command, and
  // the pose acted on is a loop (about 55 ms) old. Shorter, strafing shots clipped the Cell's edge on the side the
  // robot was moving toward
  public static double FEED_DELAY_S = 0.30;
  public static double FEED_DELAY_MAX_S = 0.30; // Latest a ball leaves: the heading must still be on target then
  // Time constant of the low-pass on the velocity the aim uses. The ball leaves FEED_DELAY_S after the decision, so
  // it needs the mean velocity over that time, not one loop's Pinpoint sample, whose sideways noise swung the aim
  // heading and the lock chased it. Cost: through a steady 0.4 m/s^2 sideways change the aim lags about 2.5 degrees;
  // the steady gate refuses faster changes. 0 is off
  public static double AIM_VELOCITY_SMOOTHING_S = 0.12;

  // Aim at the up Cell's field position once the tags have placed the robot; the tracker's Cell before that
  public static boolean AIM_AT_FIELD_CELL = true;
  // Aim at our Cell on the robot's side of the Hive (x > 0 AUDIENCE, x < 0 SCORING), not the one the tags vote up:
  // the vote flipped during runs, and the far volleys went to the far Cell, not the near one the driver meant. Which
  // Cell is right is the driver's call. False: the voted up Cell
  public static boolean AIM_AT_FRONT_CELL = true;
  public static double FRONT_CELL_SWITCH_M = 0.15; // Past x = 0 by this much before the Cell changes sides

  public static double SIDE_MARGIN_M = 0.02; // Kept clear of each side of the opening
  public static double HOOD_STOP_MARGIN_DEG = 1.0; // Targets stay this far inside the stop
  // Above this, the ball's sideways error grows too fast. 40 keeps a shot strafing at 1.2 m/s from 0.7 m (30 needed
  // 1.0 m); a 10% error in the ball's exit speed then misses sideways by about 6 cm of the opening's 20
  public static double MAX_TURNING_DEG = 40;
  /**
   * Neighbouring table cells are only blended when their hood angles are this close. Further apart
   * they are different kinds of shot, and a blend may miss, so the nearest cell is used. tools/shots/export.js
   * checks every blend in the table scores and prints the widest spread (6 now). Below it, noise flips the
   * nearest cell and the hood target jumps by whole degrees.
   */
  public static double MAX_HOOD_DIFFERENCE_TO_BLEND_DEG = 7;

  // Added to every hooded shot from the table. 0: the model, refitted to Shot Tuning's hooded balls, fires what they
  // found without one
  public static double HOODED_RPM_OFFSET = 0;
  // Aim turned anticlockwise on every shot outside the tuning tools, which set their own. Where Shot Tuning's aim
  // search and SOTM Tuning's hooded spots settled, and both tuning tools fire with it. Without it TeleOp aimed ~2 deg
  // right
  public static double HOODED_HEADING_TRIM_DEG = 2.0;
}
