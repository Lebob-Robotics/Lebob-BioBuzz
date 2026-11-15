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
  // To 0.81 m, the middle of what scored in Shot Tuning's RPM searches. On its stop the hood never scored from
  // 0.4-0.6 m, and at 0.40 m 88-94 degrees all scored at 3200 rpm but none often, so 90. From 0.95 m out the one-ball
  // points went long as volleys in the wedge, so each sits the opening's half-depth (about 90 rpm by the drag model)
  // under the slowest long ball near it; 2.3 m had no ball, the model's shape scaled. Recheck these with Shot
  // Tuning's backward spots before raising SHOT_PICK_BACKWARD_MAX_LIP_M
  public static final double[] BACKWARD_DIST_M = {0.40, 0.63, 0.73, 0.81, 0.95, 1.3, 1.8, 2.3};
  public static final double[] BACKWARD_RPM = {3200, 3180, 3230, 3390, 3370, 3370, 3460, 3800};
  public static final double[] BACKWARD_HOOD_DEG = {90, 94.3, 95.9, 97.0, 99, 99, 99, 99};
  // Closer than the table's first point, the first point's shot is used
  public static double BACKWARD_MIN_DIST_M = 0.35;
  public static double BACKWARD_MAX_SPEED_MPS = 0.3; // Toward or sideways

  // As AimConstants.HOODED_HEADING_TRIM_DEG, for the backward shot
  public static double BACKWARD_HEADING_TRIM_DEG = 1.5;
}
