package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/** Corrections for shots taken while the robot moves. Editable live from Panels. */
@Configurable
public final class SotmConstants {
  private SotmConstants() {}

  // Moving-shot corrections, scaled for fitting them to SOTM Tuning's balls (tools/analysis/sotm_report.py). 1 is the
  // model as built: the sideways speed the shooter turns to cancel, and the speed toward the Cell the table looks up
  public static double SOTM_SIDEWAYS_GAIN = 1.0;
  public static double SOTM_TOWARD_GAIN = 1.0;
  // Straight corrections per m/s of the speed the shot was looked up with, from SOTM Tuning's spot searches: launch
  // angle added for speed toward the Cell, RPM added for it, and aim turned anticlockwise for sideways speed (left +).
  // All left at 0 as within noise, and the release delay matched FEED_DELAY_S: the moving model is right as built
  public static double SOTM_HOOD_DEG_PER_TOWARD_MPS = 0;
  // Launch angle added to a moving hooded shot by its distance to the lip, in TeleOp only (the tuning tools measure
  // against the bare table). Straight between the points, held past the ends; full from MOVING_SHOT_SPEED_MPS, none at
  // rest. The found spots' hood centres run with distance, not speed, and standing balls don't show it. With it all the
  // spots' scoring windows hold the shot; without it some miss
  public static final double[] SOTM_HOOD_TRIM_DIST_M = {0.95, 1.48};
  public static final double[] SOTM_HOOD_TRIM_DEG = {-2.3, 0.7};
  public static double SOTM_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_HEADING_DEG_PER_SIDEWAYS_MPS = 0;
  public static double SOTM_BACKWARD_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS = 0;
}
