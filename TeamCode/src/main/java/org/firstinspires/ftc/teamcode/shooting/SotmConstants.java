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
  // Fitted 7 Nov to 16 finished hooded spots (0.3-0.7 m/s, 0.8-1.6 m), all left at 0 as within noise:
  //   hood -1.22 +/- 1.02 deg per m/s toward (scatter 1.35 deg; standing error by band far +0.8, mid +0.75, near -1.6)
  //   aim -0.13 +/- 0.32 deg per m/s sideways (scatter 0.41 deg), release delay 0.292 +/- 0.031 s against FEED_DELAY_S
  //   0.30: the moving model is right as built. Left out: F ARC left 0.60 at 0.98 m, whose aim (-5.25) came from 8
  //   balls left in a row at a +6 deg hood trim while the aim step was stuck at 0.25 deg
  public static double SOTM_HOOD_DEG_PER_TOWARD_MPS = 0;
  // Launch angle added to a moving hooded shot by its distance to the lip, in TeleOp only (the tuning tools measure
  // against the bare table). Straight between the points, held past the ends; full from MOVING_SHOT_SPEED_MPS, none at
  // rest. All 17 found spots' hood centres run with distance, not speed: -0.88 +/- 0.18 deg at 1.2 m, +5.53 +/- 0.76
  // deg per m (scatter 1.29 deg; adding toward -0.64 +/- 0.52, adding |sideways| -1.92 +/- 1.37: noise). Standing balls don't show it (Shot Tuning 085918 centres -2.5..+2.4 deg either side of
  // the table at every distance). With it all 16 spots' scoring windows hold the shot (worst 0.57 deg inside); without
  // it, toward-right 0.50 at 0.91 m (window -3.7..-1.5) and field -y 0.70 at 1.14 m (-4.0..-0.5) miss
  public static final double[] SOTM_HOOD_TRIM_DIST_M = {0.95, 1.48};
  public static final double[] SOTM_HOOD_TRIM_DEG = {-2.3, 0.7};
  public static double SOTM_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_HEADING_DEG_PER_SIDEWAYS_MPS = 0;
  public static double SOTM_BACKWARD_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS = 0;
}
