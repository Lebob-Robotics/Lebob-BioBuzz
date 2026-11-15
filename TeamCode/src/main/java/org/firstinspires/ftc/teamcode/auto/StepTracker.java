package org.firstinspires.ftc.teamcode.auto;

/**
 * The step clock and ball count a step plan keeps ({@link AutoSequence}, {@link CornerSequence}): when the step
 * started, when its drive arrived, how long the feeder has found no ball, and the balls shot. Volley timings are
 * {@link AutoSequence}'s.
 */
final class StepTracker {
  private double stepStartS;
  private double arrivedAtS = Double.NaN;
  private double emptySinceS = Double.NaN;
  private int ballsThisVolley = 0;
  private int ballsShot = 0;
  private boolean newStep = true;

  StepTracker(double startS) {
    stepStartS = startS;
  }

  /** First in each loop: counts a ball that left the shooter, and times how long the feeder has found none. */
  void countBalls(double nowS, boolean ballLeft, boolean feederEmpty) {
    if (ballLeft) {
      ballsThisVolley++;
      ballsShot++;
    }
    if (!feederEmpty || ballLeft) {
      emptySinceS = Double.NaN;
    } else if (Double.isNaN(emptySinceS)) {
      emptySinceS = nowS;
    }
  }

  /** Notes the first loop the step's drive is done. */
  void arrive(double nowS, boolean pathDone, boolean shooting) {
    if (pathDone && Double.isNaN(arrivedAtS)) {
      arrivedAtS = nowS;
      if (shooting) {
        emptySinceS = Double.NaN; // Empty counts from the spot, where the feeder starts feeding
      }
    }
  }

  /** The next step starts now; a shooting step starts a new volley. */
  void enter(double nowS, boolean shooting) {
    stepStartS = nowS;
    arrivedAtS = Double.NaN;
    emptySinceS = Double.NaN;
    if (shooting) {
      ballsThisVolley = 0;
    }
    newStep = true;
  }

  /** The step's drive is done. */
  boolean arrived() {
    return !Double.isNaN(arrivedAtS);
  }

  double inStepS(double nowS) {
    return nowS - stepStartS;
  }

  /** Seconds since this step's drive was done, or NaN while it is still driving. */
  double arrivedForS(double nowS) {
    return nowS - arrivedAtS;
  }

  /** At the shooting spot, and the volley finished or given up on. */
  boolean volleyEnded(double nowS) {
    if (!arrived()) {
      return false;
    }
    return volleyDone(nowS) || nowS - arrivedAtS >= AutoSequence.VOLLEY_WAIT_S;
  }

  boolean volleyDone(double nowS) {
    return ballsThisVolley >= AutoSequence.BALLS_PER_VOLLEY
        || (!Double.isNaN(emptySinceS) && nowS - emptySinceS >= AutoSequence.EMPTY_S);
  }

  /** True once after each change of step, including the first: the moment to start that step's drive. */
  boolean takeNewStep() {
    boolean was = newStep;
    newStep = false;
    return was;
  }

  int ballsShot() {
    return ballsShot;
  }
}
