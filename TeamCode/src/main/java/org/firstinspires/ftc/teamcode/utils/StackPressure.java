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
  /**
   * A new LT press clears "packed" and "full" only after LT was let go this long. Drivers tap LT 25-40 times a minute,
   * 0.1 s apart (13 Nov), so clearing on every press brought the full push back each tap.
   */
  public static double REARM_RELEASE_S = 0.5;

  /**
   * The eased intake goes back to full once its current reads under this for {@link #FREE_S}: nothing is pushing
   * back, so the stack isn't packed. Eased against a packed stack it holds 3.5-5 A; running free at half power,
   * 1.4-1.6 A (14 Nov 230333). A ball taken in reads 4.8-6.6 A for 0.4-0.5 s, which trips "packed" as one did, and
   * eased it then stayed at half power for 13 s, grinding balls in at 4-5 A.
   */
  public static double FREE_AMPS = 2.5;
  public static double FREE_S = 0.3;

  private double highSinceS = Double.NaN;
  private double lowSinceS = Double.NaN;

  /** Returns true once the eased intake has run free FREE_S. running is false whenever it isn't eased and turning. */
  public boolean runningFree(double nowS, boolean running, double amps) {
    if (!running || !(amps < FREE_AMPS) || !(amps > 0)) {
      lowSinceS = Double.NaN;
      return false;
    }
    if (Double.isNaN(lowSinceS)) {
      lowSinceS = nowS;
    }
    if (nowS - lowSinceS >= FREE_S) {
      lowSinceS = Double.NaN;
      return true;
    }
    return false;
  }

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
