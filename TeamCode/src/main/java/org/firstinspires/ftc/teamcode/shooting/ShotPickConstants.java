package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Which shot TeleOp takes: the hooded one out of the back or the backward one out of the front. Editable live from
 * Panels.
 */
@Configurable
public final class ShotPickConstants {
  private ShotPickConstants() {}

  // Lebot Teleop always takes the backward shot this close (its own distance to the lip) and the hooded one beyond.
  // Standing (6 Nov): backward 0.42-0.60 m 13/17 against hooded ~50%; the drivers chose 0.72 m as the switch. Moved to
  // 0.80 (7 Nov) so the 0.77 m shots the drivers take backward (4 of 4, Shot Tuning 080751) are backward. Hooded at the
  // table's settings scored 46 of 55 at 0.75 m over the same sessions, against 18 of 29 backward
  public static double BACKWARD_PICK_MAX_DIST_M = 0.80;
  // A backward shot that stays blocked this long, by speed or by being too close, gives way to the hooded shot
  // if that has one. Driving around the Cell at full stick held a backward pick for 30 s with no shot (5 Nov, 094211)
  public static double BACKWARD_GIVE_UP_S = 1.0;

  // Both TeleOps take whichever shot, hooded or backward, is ready soonest (8 Nov), now that Shot Tuning and SOTM Tuning
  // have calibrated both: no more backward arc. False goes back to the arc above (BACKWARD_PICK_MAX_DIST_M).
  // The time to ready is the longest of the turn, the hood's move and the wheels' change, plus BLOCKED_S if the shot
  // can't be taken yet (too fast for it). Fitted to Lebot Teleop's logs (1-7 Nov, 40 logs): 58 heading-lock turns,
  // median 0.50 s for 10-30 degrees, 0.77-0.92 s for 30-90 and 1.15 s for 90-180; 128 hood moves, median 0.23 s for
  // 4-8 degrees, 0.41 s for 15-25 and 0.47 s for 25-40. The wheels: about 4000 rpm/s at full power
  public static boolean SHOT_PICK_QUICKEST = true;
  public static double SHOT_PICK_TURN_BASE_S = 0.35;
  public static double SHOT_PICK_TURN_DEG_PER_S = 180;
  public static double SHOT_PICK_HOOD_BASE_S = 0.15;
  public static double SHOT_PICK_HOOD_DEG_PER_S = 90;
  public static double SHOT_PICK_RPM_PER_S = 4000;
  public static double SHOT_PICK_BLOCKED_S = 1.0;
  // Beyond this the hooded shot is always taken: the backward table is calibrated standing at 0.40-0.81 m only (Shot
  // Tuning: 331 balls at 0.4-0.8 m, 5 past 0.85), and SOTM Tuning has no backward passes. Its 0.95-2.3 m points
  // were never shot. Raise once Shot Tuning has searched backward spots out there
  public static double SHOT_PICK_BACKWARD_MAX_LIP_M = 0.85;

  // Straight in front of the Hive (the Cell's axis runs along field x, out from its wall) and this many degrees to
  // either side, only the backward shot is taken (13 Nov): the hooded shot's backspin affected shots from there.
  // Ignores SHOT_PICK_QUICKEST and doesn't give way to the hooded shot when blocked. 0 is off
  public static double BACKWARD_ONLY_WEDGE_DEG = 30;
  // The wedge forces the backward shot only this close (lip). The wedge never held until 14 Nov's fix, and 61 of 427
  // TeleOp balls (7-14 Nov) left from 0.9-1.1 m in front, against the rear wall: forced backward there they'd use
  // table points never shot (past 0.95 m). 0.95 takes in 230333's bounced 0.90 m volley. Past it, as before
  public static double BACKWARD_WEDGE_MAX_LIP_M = 0.95;
}
