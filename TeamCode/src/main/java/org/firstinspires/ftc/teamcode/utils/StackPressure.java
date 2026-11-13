package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tells when the intake is pushing a packed stack: its current holds high while loading, because the balls ahead
 * can't move. Pushed on, the stack forces the first ball past the backward-held feeder into the spinning
 * flywheels, which shoot it (13 Nov 225900 at 73.5 s: 0.6 s at 4.4-4.8 A, then a 400 RPM dip). Loading free,
 * the intake draws 1.8-2.2 A, and taking a ball in reads over 4.2 A for under 0.4 s.
 */
@Configurable
public class StackPressure {
  public static double PACKED_AMPS = 4.2;
  public static double PACKED_S = 0.4;
  /** Intake power while packed: enough to keep the last ball in, about half the push. */
  public static double PACKED_INTAKE_POWER = 0.5;

  private double highSinceS = Double.NaN;

  /** Returns true on the loop the stack is seen packed. watching is false whenever the intake isn't loading. */
  public boolean update(double nowS, boolean watching, double amps) {
    if (!watching || !(amps >= PACKED_AMPS)) {
      highSinceS = Double.NaN;
      return false;
    }
    if (Double.isNaN(highSinceS)) {
      highSinceS = nowS;
    }
    if (nowS - highSinceS >= PACKED_S) {
      highSinceS = Double.NaN;
      return true;
    }
    return false;
  }
}
