package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Live shooter tuning from Panels. The velocity PIDF and IDLE_RPM are in ShooterSubsystem, also
 * editable in Panels. The stall guard limits are deliberately not tunable.
 */
@Configurable
public final class ShooterTuning {
  private ShooterTuning() {}

  /**
   * Flywheel RPM held while RT or Y is down, in closed loop. 0 leaves the normal behaviour: RT uses
   * the shot table (or the top speed with no Cell in view) and Y runs open loop at full power.
   * Clamped to the top speed the motors can reach.
   */
  public static double RPM_OVERRIDE = 0;

  /** Share of the target RPM the wheels must reach before RT feeds when the Cell is not in view. */
  public static double MANUAL_SPEED_SHARE = 0.85;

  /** Feed anyway after RT has been held this long without the Cell, if the wheels never get there. */
  public static double MANUAL_MAX_SPINUP_S = 1.0;
}
