package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The backward shot (tools/shots/backwards-calibration.json): out of the robot's front, the other way to the cameras,
 * with the hood on its lower stop so the ball leaves over the top. Taped from the flywheel axle to the near lip with
 * the robot still, so there is no speed axis: it is only taken while the robot is nearly stopped. Editable live from
 * Panels.
 */
@Configurable
public final class BackwardShotConstants {
  private BackwardShotConstants() {}

  public static boolean BACKWARD_SHOTS_ON = true;
  // 0.40 m (Shot Tuning, 6 Nov 082517): on its stop the hood never scored from 0.4-0.6 m (0 of 7 at 3400-3600 rpm,
  // and the G1 grid 0 of 3 at 0.6 m, onto the Cell's bottom); 8 degrees up at 3200 rpm scored 4 of 5, 3150-3300
  // 8 of 13. Shot Tuning 091010: 0.42 m at 90.3 degrees 8 of 10 against 0 of 2 at 91.3; 0.60 m 3270 rpm 94.0 degrees
  // 5 of 7; 0.72-0.74 m as interpolated (3310-3320, 95.5-95.9) 14 of 25, misses unmarked. Shot Tuning 080751 (7 Nov):
  // 0.77 m as interpolated (3320-3330, 95.9-96.5) 4 of 4, 23 degrees off the Cell's axis; 47-51 degrees off,
  // 0.74-0.85 m scored 3 of 14 at 3370-3470 rpm and 94.7-97.7 degrees.
  // Shot Tuning 085918 (7 Nov), each spot's RPM search at the old table's angle, the middle of what scored:
  // 0.63 m 3000-3360 at 94.3 (7 of 7 at 3180), 0.73-0.74 m 3200-3340 and 3020-3350 at 95.8-95.9, 0.76 m 3280-3310
  // at 96.5 (the line between 0.73 and 0.81 gives 3300 there), 0.81 m 3350-3420 at 97.0. 0.40 m: 88-94 degrees all
  // scored at 3200 but none often (14 of 52 over the session), so 90 stays
  // 0.95 m out (13 Nov): the 3 Nov one-ball points went long as volleys in the wedge. Lebot Teleop's 8 far backward
  // balls (173844-225900, hood 97.6-98.2) left at 3505 rpm from 1.53 m, 3566-3597 from 1.52-1.59, 3628 from 1.85 and
  // 3782-3812 from 1.94, all long. The drag model fitted to Shot Tuning's 179 marked backward balls (0.4-0.9 m) puts
  // the opening's half-depth at about 90 rpm out there, so each point sits at least that far under the slowest long
  // ball near it: 1.53 m 3411 (was 3592), 1.85 m 3494 (3750), 1.94 m 3555 (3836). 2.3 m: no ball, the model's
  // shape scaled to 1.85 m. Recheck these with Shot Tuning's backward spots before raising SHOT_PICK_BACKWARD_MAX_LIP_M
  public static final double[] BACKWARD_DIST_M = {0.40, 0.63, 0.73, 0.81, 0.95, 1.3, 1.8, 2.3};
  public static final double[] BACKWARD_RPM = {3200, 3180, 3230, 3390, 3370, 3370, 3460, 3800};
  public static final double[] BACKWARD_HOOD_DEG = {90, 94.3, 95.9, 97.0, 99, 99, 99, 99};
  // Closer than the table's first point, the first point's shot is used
  public static double BACKWARD_MIN_DIST_M = 0.35;
  public static double BACKWARD_MAX_SPEED_MPS = 0.3; // Toward or sideways

  // As AimConstants.HOODED_HEADING_TRIM_DEG, for the backward shot
  public static double BACKWARD_HEADING_TRIM_DEG = 1.5;
}
