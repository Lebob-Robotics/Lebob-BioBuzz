package org.firstinspires.ftc.teamcode.auto;

/**
 * The autonomous's steps and when each ends, without hardware. Choon Kai's plan: shoot the 4 preloads driving to
 * the Garden, sweep it, shoot driving to our wall's Flower, take its Pollen, shoot driving to the rear Flower, take
 * its Pollen, shoot driving to the loading zone and park. Shots are fired while driving, from wherever the robot is.
 *
 * <p>Not away from the first Flower by {@link #SKIP_SECOND_FLOWER_S}: the second Flower is skipped and the robot
 * goes to park. Whatever it is doing at {@link #PARK_BY_S}, it goes to park.
 */
public final class AutoSequence {
  public enum Step {
    DRIVE_TO_GARDEN(true), SWEEP_GARDEN(false), DRIVE_TO_WALL_FLOWER(true), INTAKE_WALL_FLOWER(false),
    DRIVE_TO_REAR_FLOWER(true), INTAKE_REAR_FLOWER(false), DRIVE_TO_PARK(true), PARKED(false);

    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  public static double SKIP_SECOND_FLOWER_S = 16.0;
  /** Latest start of the drive to park: about 4 s of it, a second to settle, and margin to 30 s. */
  public static double PARK_BY_S = 24.0;
  public static int BALLS_PER_VOLLEY = 4;
  /** The feeder found no ball for this long: the robot is empty. */
  public static double EMPTY_S = 0.6;
  /** Longest wait at a drive's end for its volley to finish. */
  public static double VOLLEY_WAIT_S = 2.0;
  public static double GARDEN_SWEEP_MAX_S = 3.0;
  public static double FLOWER_INTAKE_S = 4.0;

  private Step step = Step.DRIVE_TO_GARDEN;
  private double stepStartS;
  private double arrivedAtS = Double.NaN;
  private double emptySinceS = Double.NaN;
  private int ballsThisVolley = 0;
  private int ballsShot = 0;
  private boolean skipSecondFlower = false;
  private boolean newStep = true;

  public AutoSequence(double startS) {
    stepStartS = startS;
  }

  /**
   * One loop.
   *
   * @param pathDone    the drive for this step has reached its end (Pedro is holding there)
   * @param ballLeft    a ball left the shooter this loop
   * @param feederEmpty the feed gate found no ball on its last pulse
   */
  public void update(double nowS, boolean pathDone, boolean ballLeft, boolean feederEmpty) {
    update(nowS, pathDone, ballLeft, feederEmpty, false);
  }

  /** As above. @param stackFull the robot is full: a Flower step ends at once instead of after FLOWER_INTAKE_S */
  public void update(double nowS, boolean pathDone, boolean ballLeft, boolean feederEmpty, boolean stackFull) {
    if (ballLeft) {
      ballsThisVolley++;
      ballsShot++;
    }
    if (!feederEmpty || ballLeft) {
      emptySinceS = Double.NaN;
    } else if (Double.isNaN(emptySinceS)) {
      emptySinceS = nowS;
    }
    if (step.ordinal() <= Step.INTAKE_WALL_FLOWER.ordinal() && nowS >= SKIP_SECOND_FLOWER_S) {
      skipSecondFlower = true;
    }
    if (step.ordinal() < Step.DRIVE_TO_PARK.ordinal() && nowS >= PARK_BY_S) {
      enter(Step.DRIVE_TO_PARK, nowS);
      return;
    }
    if (pathDone && Double.isNaN(arrivedAtS)) {
      arrivedAtS = nowS;
    }
    double inStepS = nowS - stepStartS;
    switch (step) {
      case DRIVE_TO_GARDEN:
        if (driveDone(nowS)) {
          enter(Step.SWEEP_GARDEN, nowS);
        }
        break;
      case SWEEP_GARDEN:
        if (pathDone || inStepS >= GARDEN_SWEEP_MAX_S) {
          enter(Step.DRIVE_TO_WALL_FLOWER, nowS);
        }
        break;
      case DRIVE_TO_WALL_FLOWER:
        if (driveDone(nowS)) {
          enter(Step.INTAKE_WALL_FLOWER, nowS);
        }
        break;
      case INTAKE_WALL_FLOWER:
        if (stackFull || inStepS >= FLOWER_INTAKE_S) {
          enter(skipSecondFlower ? Step.DRIVE_TO_PARK : Step.DRIVE_TO_REAR_FLOWER, nowS);
        }
        break;
      case DRIVE_TO_REAR_FLOWER:
        if (driveDone(nowS)) {
          enter(Step.INTAKE_REAR_FLOWER, nowS);
        }
        break;
      case INTAKE_REAR_FLOWER:
        if (stackFull || inStepS >= FLOWER_INTAKE_S) {
          enter(Step.DRIVE_TO_PARK, nowS);
        }
        break;
      case DRIVE_TO_PARK:
        if (driveDone(nowS)) {
          enter(Step.PARKED, nowS);
        }
        break;
      case PARKED:
        break;
    }
  }

  /** At the drive's end, and the volley finished or given up on. */
  private boolean driveDone(double nowS) {
    if (Double.isNaN(arrivedAtS)) {
      return false;
    }
    return volleyDone(nowS) || nowS - arrivedAtS >= VOLLEY_WAIT_S;
  }

  private boolean volleyDone(double nowS) {
    return ballsThisVolley >= BALLS_PER_VOLLEY || (!Double.isNaN(emptySinceS) && nowS - emptySinceS >= EMPTY_S);
  }

  private void enter(Step next, double nowS) {
    step = next;
    stepStartS = nowS;
    arrivedAtS = Double.NaN;
    emptySinceS = Double.NaN;
    if (next.shooting) {
      ballsThisVolley = 0;
    }
    newStep = true;
  }

  public Step step() {
    return step;
  }

  /** True once after each change of step, including the first: the moment to start that step's drive. */
  public boolean takeNewStep() {
    boolean was = newStep;
    newStep = false;
    return was;
  }

  /** Shooting while a drive's volley isn't finished. Parked, it keeps trying until the end. */
  public boolean shoot(double nowS) {
    if (step == Step.PARKED) {
      return ballsThisVolley < BALLS_PER_VOLLEY && !volleyDone(nowS);
    }
    return step.shooting && !volleyDone(nowS);
  }

  public boolean intake() {
    return step == Step.SWEEP_GARDEN || step == Step.INTAKE_WALL_FLOWER || step == Step.INTAKE_REAR_FLOWER;
  }

  public boolean isSkippingSecondFlower() {
    return skipSecondFlower;
  }

  public int ballsShot() {
    return ballsShot;
  }
}
