package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

/** SOTM Tuning's settings, editable live from Panels. */
@Configurable
public final class SotmTuningSettings {
  private SotmTuningSettings() {}

  // Drive power for a speed: v = SPEED_PER_POWER_MPS * (power - POWER_AT_REST), fitted to logged speeds
  public static double SPEED_PER_POWER_MPS = 1.25;
  public static double POWER_AT_REST = 0.13;
  public static double TRACK_GAIN_PER_S = 2.5; // Speed added per metre off the plan's line
  // Along the line, the plan's clock waits instead: chasing a start lag at TRACK_GAIN_PER_S surged a slow pass and
  // the steady gate never opened
  public static double TRACK_ALONG_GAIN_PER_S = 1.0; // Speed added per metre behind the plan along its line
  public static double LAG_HOLD_M = 0.15; // This far behind along the line the plan's clock stops; half as far, half rate
  public static double MAX_EXTRA_S = 3.0; // The pass stops after its length plus this, however far the clock got
  public static double TRACK_SPEED_GAIN = 0.5; // Share of the speed shortfall added
  // Share of the excess speed taken off when faster than the plan along its line. Too low and the stick never
  // reverses: the plan brakes while the robot rolls on into the fence check
  public static double TRACK_BRAKE_GAIN = 1.5;
  // Drives to the plan's speed this far ahead, so the wheels start a speed change when the plan does
  public static double TRACK_LEAD_S = 0.2;
  public static double MAX_CORRECTION_MPS = 0.3;
  public static double MIN_SPEED_MPS = 0.03; // Asked for less, the drive stops rather than creeping at the least power
  public static double ABORT_TRACK_M = 0.35; // Further than this off the plan's point now, the pass stops

  public static double READY_MIN_S = 1.0; // Aims at the start this long at least
  public static double READY_MAX_S = 3.0;
  public static double READY_HEADING_DEG = 2.0;

  /** 0: each pass's own (steady passes keep the match's gates, mid-change passes fire without them); 1: the match's; 2: none. */
  public static int GATES = 0;
  // Passes fire moving inside the match's limit of the lip (MIN_MOVING_SHOT_LIP_M) too, unless GATES is 1, so their
  // balls show whether those shots score
  public static boolean FIRE_CLOSE_MOVING = true;
  // Set from Panels for one press of B (the next pass, or stop the one running) without the gamepad; cleared at once
  public static boolean PRESS_B = false;
  // After each pass it drives here and holds, so a ball can be loaded by hand. Field metres, robot centre. The
  // default is the -x, -y corner (Panels' bottom left) with about 4 in to each wall: the robot is 18 in square, so its
  // centre can't get closer than 0.23 m to the wall. Off: it stays where the pass ended
  public static boolean RELOAD_BETWEEN = true;
  public static double RELOAD_X_M = -1.50;
  public static double RELOAD_Y_M = -1.50;
}
