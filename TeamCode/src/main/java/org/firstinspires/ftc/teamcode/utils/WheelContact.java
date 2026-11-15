package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayDeque;

/**
 * Tells when a loading ball reaches the spinning flywheels: their speed falls sharply while the target holds
 * still. With the beam breaks gone, that touch is the sign the stack is full. Loading on after it pushed the
 * first ball past the backward-held feeder and shot it (13 Nov: 79 such dips over 917 s of loading).
 */
@Configurable
public class WheelContact {
  public static double WINDOW_S = 0.25;
  /** A fall this big within the window is a ball, not speed control. Shots fed at speed dip a median 307 RPM. */
  public static double DIP_RPM = 300;
  /**
   * The window's fastest speed must be this near the target either way, so spinning up, or settling back from an
   * overshoot (191420 at 10.1 s: 3874 on a 3563 target), isn't a touch.
   */
  public static double NEAR_TARGET_RPM = 250;
  /**
   * Nor may it be more than this over the target: wheels coming down from above it aren't touched by a ball. The
   * Scoring Auto's target stepped from 3942 to 3733 while it drove, the wheels fell from 3996 to 3689, and that read
   * as a full stack, so it ejected at the rear Flower for 3.2 s (13 Nov 224556). At rest they wander up to 66 over
   */
  public static double ABOVE_TARGET_RPM = 100;
  /** A target moving more than this within the window isn't held still. */
  public static double TARGET_STEADY_RPM = 100;
  /** And the target must have held within TARGET_STEADY_RPM this long, window included: the wheels have settled on it. */
  public static double TARGET_HELD_S = 0.75;

  private final ArrayDeque<double[]> window = new ArrayDeque<>(); // {t, rpm, target}
  private double heldTargetRpm = Double.NaN;
  private double heldSinceS = Double.NaN;

  /** Returns true on the loop a touch is seen. watching is false whenever loading or steady aiming isn't on. */
  public boolean update(double nowS, boolean watching, double rpm, double targetRpm) {
    if (!Double.isNaN(targetRpm)
        && (Double.isNaN(heldTargetRpm) || Math.abs(targetRpm - heldTargetRpm) > TARGET_STEADY_RPM)) {
      heldTargetRpm = targetRpm;
      heldSinceS = nowS;
    }
    if (!watching || Double.isNaN(rpm) || Double.isNaN(targetRpm)) {
      window.clear();
      return false;
    }
    window.addLast(new double[] {nowS, rpm, targetRpm});
    while (!window.isEmpty() && nowS - window.peekFirst()[0] > WINDOW_S) {
      window.removeFirst();
    }
    double peak = Double.NEGATIVE_INFINITY, lowTarget = Double.POSITIVE_INFINITY, highTarget = Double.NEGATIVE_INFINITY;
    for (double[] s : window) {
      peak = Math.max(peak, s[1]);
      lowTarget = Math.min(lowTarget, s[2]);
      highTarget = Math.max(highTarget, s[2]);
    }
    boolean touch = nowS - heldSinceS >= TARGET_HELD_S && highTarget - lowTarget <= TARGET_STEADY_RPM
        && targetRpm - peak <= NEAR_TARGET_RPM && peak - targetRpm <= ABOVE_TARGET_RPM && peak - rpm >= DIP_RPM;
    if (touch) {
      window.clear();
    }
    return touch;
  }
}
