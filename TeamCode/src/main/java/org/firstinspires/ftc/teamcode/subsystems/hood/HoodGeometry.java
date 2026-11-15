package org.firstinspires.ftc.teamcode.subsystems.hood;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.shooting.ShooterGeometry;

/**
 * The hood's encoders and travel. The REV Through Bore and the motor's own encoder are both on the pinion, 1:1,
 * and the pinion turns the hood through a 10:1 reduction. Hood ticks are REV ticks above the lower stop.
 */
@Configurable
public final class HoodGeometry {
  private HoodGeometry() {}

  public static final double REV_COUNTS_PER_TURN = 8192;
  public static final double MOTOR_COUNTS_PER_TURN = 384.5; // 435 RPM 5203 output shaft
  /** Motor encoder ticks per REV tick: 384.5 vs 8192 counts per pinion turn, 1:1. */
  public static final double MOTOR_PER_REV = MOTOR_COUNTS_PER_TURN / REV_COUNTS_PER_TURN;

  /** Launch angle at the lower hard stop, where the encoder is zeroed. */
  public static double ZERO_ANGLE_DEG = 99.0;
  /** Launch angle at the top of the hood's travel; sets the upper soft limit. */
  public static double MIN_ANGLE_DEG = ShooterGeometry.HOOD_TOP_STOP_DEG;
  /** Encoder ticks per degree of launch angle: 8192 ticks/rev through the 10:1. */
  public static double TICKS_PER_DEG = 8192.0 * 10.0 / 360.0;
}
