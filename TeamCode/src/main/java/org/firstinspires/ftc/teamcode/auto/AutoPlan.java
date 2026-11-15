package org.firstinspires.ftc.teamcode.auto;

/** A step plan without hardware ({@link AutoSequence}, {@link CornerSequence}), as an auto OpMode runs it. */
public interface AutoPlan<S extends Enum<S>> {
  /**
   * One loop.
   *
   * @param pathDone    the step's drive has reached its spot (for a step that turns in place, and faces its way)
   * @param ballLeft    a ball left the shooter this loop
   * @param feederEmpty the feed gate found no ball on its last pulse
   * @param stackFull   the robot is full: a Flower step ends at once
   */
  void update(double nowS, boolean pathDone, boolean ballLeft, boolean feederEmpty, boolean stackFull);

  S step();

  /** True once after each change of step, including the first: the moment to start that step's drive. */
  boolean takeNewStep();

  /** Stopped at a shooting spot with its volley not finished, or parked. */
  boolean shoot(double nowS);

  boolean intake();

  /** The indexer stops (or, with the auto's reverse on, runs backward) for now. */
  boolean indexerPaused(double nowS);

  /** The Pollen arm is down. */
  boolean armDown();

  /** Holding a spot until the robot is on it and at its heading. */
  boolean isFacingStep();

  int ballsShot();

  /** Seconds since this step's drive was done, or NaN while it is still driving. */
  double arrivedForS(double nowS);
}
