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
  public static double RPM_OVERRIDE = 4440;

  /** Both wheels must be within this many RPM of the target before RT starts feeding. */
  public static double MANUAL_RPM_TOLERANCE = 100;
}
