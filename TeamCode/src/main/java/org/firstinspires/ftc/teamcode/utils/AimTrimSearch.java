package org.firstinspires.ftc.teamcode.utils;

/**
 * Shot Tuning's aim trim, from the balls marked left or right: each one turns the aim the other way by a step, halved
 * each time the misses swap sides. Anticlockwise (left) positive, as Robot#setHeadingTrimDeg.
 */
public final class AimTrimSearch {
  private final double startStepDeg;
  private final double minStepDeg;
  private double trimDeg;
  private double stepDeg;
  private int lastSide; // +1 the last side miss went left, -1 right, 0 none yet

  public AimTrimSearch(double stepDeg, double minStepDeg) {
    this.startStepDeg = stepDeg;
    this.minStepDeg = minStepDeg;
    this.stepDeg = stepDeg;
  }

  /** Turns the aim away from a left or right miss. Returns the change, degrees. */
  public double record(BallMark mark) {
    if (mark.in || mark.bounced || !mark.hasSide()) {
      return 0;
    }
    int side = mark.left ? 1 : -1;
    if (lastSide != 0 && side != lastSide) {
      stepDeg = Math.max(minStepDeg, stepDeg / 2);
    }
    lastSide = side;
    double change = -side * stepDeg;
    trimDeg += change;
    return change;
  }

  /** The driver set the trim by hand: searching goes on from there, at the full step. */
  public void set(double deg) {
    trimDeg = deg;
    stepDeg = startStepDeg;
    lastSide = 0;
  }

  public double trimDeg() { return trimDeg; }
  public double stepDeg() { return stepDeg; }
}
