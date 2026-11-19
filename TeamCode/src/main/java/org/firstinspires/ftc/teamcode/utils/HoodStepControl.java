package org.firstinspires.ftc.teamcode.utils;

/** Physical hood lift from its lower stop: RIGHT raises 2 degrees, LEFT lowers 2 per press. */
public final class HoodStepControl {
  public static final double STEP_DEG = 2;
  private boolean wasRight;
  private boolean wasLeft;

  public double update(double currentLiftDeg, boolean right, boolean left, double maxLiftDeg) {
    double change = (right && !wasRight ? STEP_DEG : 0) - (left && !wasLeft ? STEP_DEG : 0);
    wasRight = right;
    wasLeft = left;
    return Math.max(0, Math.min(maxLiftDeg, currentLiftDeg + change));
  }
}
