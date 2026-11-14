package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;

/**
 * The autonomous's steps and when each ends, without hardware. The plan (14 Nov):
 *
 * <ol>
 *   <li>Drive to the Hive and shoot the 4 preloads into the up Cell, which tips it.
 *   <li>Take the Garden: turn intake first along the Pollen line, clear of the wall; slide sideways onto the
 *       audience wall; sweep along it against it.
 *   <li>Drive to the other Cell, now up, and shoot.
 *   <li>Take our wall's Flower, then shoot.
 *   <li>Take the rear Flower, then park; parked, it shoots what it holds.
 * </ol>
 *
 * <p>The robot holds 4, so each pickup is shot before the next. Every volley is shot stopped, from a spot straight
 * out from the up Cell ({@link AutoField#shotSpot}); a drive only drives.
 *
 * <p>A Flower is taken with the Pollen arm (the ramp): the drive stops short of it, the robot faces it and lowers
 * the arm, drives in with the intake on, backs out to where it stopped, and raises the arm before driving on.
 *
 * <p>Done shooting after the first Flower later than {@link #SKIP_SECOND_FLOWER_S}: the second Flower is skipped.
 * Whatever it is doing at {@link #PARK_BY_S}, it goes to park; at a Flower it starts backing out
 * {@link #FLOWER_EXIT_S} before that, so the arm is up by then.
 */
public final class AutoSequence {
  public enum Step {
    SHOOT_PRELOAD(true),
    DRIVE_TO_GARDEN(false), FACE_GARDEN(false), ONTO_GARDEN_WALL(false), SWEEP_GARDEN(false),
    SHOOT_GARDEN(true),
    DRIVE_TO_WALL_FLOWER(false), LOWER_ARM_WALL(false), INTAKE_WALL_FLOWER(false), BACK_FROM_WALL_FLOWER(false),
    RAISE_ARM_WALL(false),
    SHOOT_WALL_FLOWER(true),
    DRIVE_TO_REAR_FLOWER(false), LOWER_ARM_REAR(false), INTAKE_REAR_FLOWER(false), BACK_FROM_REAR_FLOWER(false),
    RAISE_ARM_REAR(false),
    DRIVE_TO_PARK(false), PARKED(true);

    /** A volley is shot here, once the robot has stopped at the step's spot. */
    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  /** The rear Flower after the first Flower's volley: off, straight to park instead (14 Nov). */
  public static boolean SECOND_FLOWER = false;
  /**
   * Done shooting after the first Flower later than this, the second Flower doesn't fit before PARK_BY_S: its drive
   * (about 2.5 s), the arm, 1 s in it, backing out by FLOWER_EXIT_S before PARK_BY_S.
   */
  public static double SKIP_SECOND_FLOWER_S = 20.0;
  /**
   * Latest start of the drive to park: about 1.5 s from the rear Flower, 3 s from across the field, parked before
   * AUTO ends at 30 s.
   */
  public static double PARK_BY_S = 26.0;
  public static int BALLS_PER_VOLLEY = 4;
  /**
   * The feeder found no ball for this long: the robot is empty. 0.6 s gave up after one empty pulse, 14 Nov 220630:
   * the Flower volley quit after 1.1 s of feeding with its balls still coming up. 2 s lets the indexer's retry pulse in.
   */
  public static double EMPTY_S = 2.0;
  /**
   * Longest at a shooting spot for the volley to finish. 3 s cut the preloads off after 2 balls on 14 Nov 212840:
   * the first left 2 s after stopping, and the 2 left in blocked the Garden's intake.
   */
  public static double VOLLEY_WAIT_S = 4.0;
  /** 0.6 m along the wall at half power: 0.43 m took 1.9 s on 14 Nov. */
  public static double GARDEN_SWEEP_MAX_S = 3.5;
  /** Longest to turn to the Pollen before the sweep; then it sweeps anyway, turning on the way. */
  public static double FACE_GARDEN_MAX_S = 1.5;
  /** Longest to slide onto the audience wall; then it sweeps anyway, closing on the wall on the way. */
  public static double ONTO_WALL_MAX_S = 1.5;
  /** Longest in a Flower; 14 Nov the drive in took 1.2 s, then it sat 2.9 s in it, empty. */
  public static double FLOWER_INTAKE_S = 3.0;
  /** In the Flower, once the drive in is done, this long for the last balls to come in. */
  public static double FLOWER_DWELL_S = 1.0;
  /**
   * In a Flower, once driven in, the robot wiggles, turning this far either side of facing it, for the intake to
   * take every ball. 12 degrees was too much (14 Nov). Guesses: tune on the field.
   */
  public static double WIGGLE_RAD = Math.toRadians(7);
  /** One wiggle, left and right and back: two in the dwell. */
  public static double WIGGLE_PERIOD_S = 0.5;
  /** The arm's move between stowed and deployed: about 130 degrees on the REV servo. */
  public static double ARM_MOVE_S = 0.4;
  /** Longest to face a Flower from the stop short of it; then the arm is lowered anyway as the robot drives in. */
  public static double FACE_FLOWER_MAX_S = 2.0;
  /** Longest to back out of a Flower. */
  public static double BACK_OUT_MAX_S = 2.0; // 0.28 m took 0.8 s on 14 Nov
  /** A Flower is left this long before PARK_BY_S: backing out and raising the arm. */
  public static double FLOWER_EXIT_S = 2.0;

  private Step step = Step.SHOOT_PRELOAD;
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
   * @param pathDone    the step's drive has reached its spot (for a step that turns in place, and faces its way)
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
    if (inFlower() && nowS >= PARK_BY_S - FLOWER_EXIT_S) {
      leaveFlower(nowS);
      return;
    }
    // Backing out and raising the arm finish first: they take under 2.5 s
    if (step.ordinal() < Step.DRIVE_TO_PARK.ordinal() && nowS >= PARK_BY_S && !leavingFlower()) {
      enter(Step.DRIVE_TO_PARK, nowS);
      return;
    }
    if (pathDone && Double.isNaN(arrivedAtS)) {
      arrivedAtS = nowS;
      if (step.shooting) {
        emptySinceS = Double.NaN; // Empty counts from the spot, where the feeder starts feeding
      }
    }
    double inStepS = nowS - stepStartS;
    boolean late = nowS >= PARK_BY_S - FLOWER_EXIT_S; // No time for a Flower
    switch (step) {
      case SHOOT_PRELOAD:
        if (volleyEnded(nowS)) {
          enter(Step.DRIVE_TO_GARDEN, nowS);
        }
        break;
      case DRIVE_TO_GARDEN:
        if (pathDone) {
          enter(Step.FACE_GARDEN, nowS);
        }
        break;
      case FACE_GARDEN: // pathDone here: clear of the wall and facing along the Pollen line
        if (pathDone || inStepS >= FACE_GARDEN_MAX_S) {
          enter(Step.ONTO_GARDEN_WALL, nowS);
        }
        break;
      case ONTO_GARDEN_WALL: // pathDone here: against the wall at the line's start
        if (pathDone || inStepS >= ONTO_WALL_MAX_S) {
          enter(Step.SWEEP_GARDEN, nowS);
        }
        break;
      case SWEEP_GARDEN:
        if (pathDone || inStepS >= GARDEN_SWEEP_MAX_S) {
          enter(Step.SHOOT_GARDEN, nowS);
        }
        break;
      case SHOOT_GARDEN:
        if (volleyEnded(nowS)) {
          enter(late ? Step.DRIVE_TO_PARK : Step.DRIVE_TO_WALL_FLOWER, nowS);
        }
        break;
      case DRIVE_TO_WALL_FLOWER:
        if (pathDone) {
          enter(late ? Step.SHOOT_WALL_FLOWER : Step.LOWER_ARM_WALL, nowS);
        }
        break;
      case DRIVE_TO_REAR_FLOWER:
        if (pathDone) {
          enter(late ? Step.DRIVE_TO_PARK : Step.LOWER_ARM_REAR, nowS);
        }
        break;
      case LOWER_ARM_WALL:
      case LOWER_ARM_REAR:
        // pathDone here: stopped short of the Flower and facing it. The arm goes down then
        if ((!Double.isNaN(arrivedAtS) && nowS - arrivedAtS >= ARM_MOVE_S) || inStepS >= FACE_FLOWER_MAX_S) {
          enter(step == Step.LOWER_ARM_WALL ? Step.INTAKE_WALL_FLOWER : Step.INTAKE_REAR_FLOWER, nowS);
        }
        break;
      case INTAKE_WALL_FLOWER:
      case INTAKE_REAR_FLOWER:
        boolean dwelt = !Double.isNaN(arrivedAtS) && nowS - arrivedAtS >= FLOWER_DWELL_S;
        if (stackFull || dwelt || inStepS >= FLOWER_INTAKE_S) {
          leaveFlower(nowS);
        }
        break;
      case BACK_FROM_WALL_FLOWER:
      case BACK_FROM_REAR_FLOWER:
        if (pathDone || inStepS >= BACK_OUT_MAX_S) {
          enter(step == Step.BACK_FROM_WALL_FLOWER ? Step.RAISE_ARM_WALL : Step.RAISE_ARM_REAR, nowS);
        }
        break;
      case RAISE_ARM_WALL:
        if (inStepS >= ARM_MOVE_S) {
          enter(Step.SHOOT_WALL_FLOWER, nowS);
        }
        break;
      case RAISE_ARM_REAR:
        if (inStepS >= ARM_MOVE_S) {
          enter(Step.DRIVE_TO_PARK, nowS);
        }
        break;
      case SHOOT_WALL_FLOWER:
        if (volleyEnded(nowS)) {
          skipSecondFlower = !SECOND_FLOWER || nowS >= SKIP_SECOND_FLOWER_S;
          enter(skipSecondFlower ? Step.DRIVE_TO_PARK : Step.DRIVE_TO_REAR_FLOWER, nowS);
        }
        break;
      case DRIVE_TO_PARK:
        if (pathDone) {
          enter(Step.PARKED, nowS);
        }
        break;
      case PARKED:
        break;
    }
  }

  private boolean inFlower() {
    return step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER
        || step == Step.LOWER_ARM_REAR || step == Step.INTAKE_REAR_FLOWER;
  }

  /** Out of a Flower: backs out if the arm is down, else straight on to what follows it. */
  private void leaveFlower(double nowS) {
    boolean wall = step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER;
    if (!armDown()) {
      enter(wall ? Step.SHOOT_WALL_FLOWER : Step.DRIVE_TO_PARK, nowS);
    } else {
      enter(wall ? Step.BACK_FROM_WALL_FLOWER : Step.BACK_FROM_REAR_FLOWER, nowS);
    }
  }

  private boolean leavingFlower() {
    return step == Step.BACK_FROM_WALL_FLOWER || step == Step.RAISE_ARM_WALL
        || step == Step.BACK_FROM_REAR_FLOWER || step == Step.RAISE_ARM_REAR;
  }

  /** At the shooting spot, and the volley finished or given up on. */
  private boolean volleyEnded(double nowS) {
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

  /** Shooting: stopped at a shooting spot, its volley not finished. Parked, until the end. */
  public boolean shoot(double nowS) {
    // Parked, it keeps on to the end, for any ball still stuck: the indexer's retry pulses free it
    return step.shooting && !Double.isNaN(arrivedAtS) && (step == Step.PARKED || !volleyDone(nowS));
  }

  /** The intake runs at the Garden, at the Flowers from lowering the arm to raising it, and through each volley. */
  public boolean intake() {
    switch (step) {
      case SHOOT_PRELOAD: // Once stopped only, for the 4th ball in the intake: on the way, with the wheels spinning,
        return !Double.isNaN(arrivedAtS); // the full stack pushed a ball through the held feeder (14 Nov 220630)
      case PARKED: // Once there, pushing any ball left in the intake up (against our wall: nothing to pick up)
        return !Double.isNaN(arrivedAtS);
      case SHOOT_GARDEN: // The whole volley, drive and shots. 14 Nov 214244 the Garden's balls sat in its mouth,
      case SHOOT_WALL_FLOWER: // switched off as the drive began, and the volley shot none; 214945, off once
        return true; // stopped, every volley left its last balls in
      case FACE_GARDEN:
      case ONTO_GARDEN_WALL:
      case SWEEP_GARDEN:
      case LOWER_ARM_WALL:
      case INTAKE_WALL_FLOWER:
      case BACK_FROM_WALL_FLOWER:
      case RAISE_ARM_WALL:
      case LOWER_ARM_REAR:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_REAR_FLOWER:
      case RAISE_ARM_REAR:
        return true;
      default:
        return false;
    }
  }

  /**
   * The Pollen arm is down: from facing the Flower (or giving up on facing it) until it backs out. Up everywhere
   * else, so it is never down on a drive between places.
   */
  public boolean armDown() {
    switch (step) {
      case LOWER_ARM_WALL:
      case LOWER_ARM_REAR:
        return !Double.isNaN(arrivedAtS);
      case INTAKE_WALL_FLOWER:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_WALL_FLOWER:
      case BACK_FROM_REAR_FLOWER:
        return true;
      default:
        return false;
    }
  }

  /**
   * Holding a spot until the robot is on it and at its heading: turning to the Garden's line, onto its wall, or to a
   * Flower before the arm goes down.
   */
  public boolean isFacingStep() {
    return step == Step.FACE_GARDEN || step == Step.ONTO_GARDEN_WALL || step == Step.LOWER_ARM_WALL
        || step == Step.LOWER_ARM_REAR;
  }

  public boolean isSkippingSecondFlower() {
    return skipSecondFlower;
  }

  public int ballsShot() {
    return ballsShot;
  }

  /**
   * The Cell a shooting step shoots at, by the team's plan: the preloads at the Cell up at the start, which tips it,
   * then every later volley at the other one. Not from balls counted.
   */
  public static Cell plannedCell(Cell first, Step shootingStep) {
    return shootingStep == Step.SHOOT_PRELOAD ? first : opposite(first);
  }

  public static Cell opposite(Cell cell) {
    return cell == Cell.AUDIENCE ? Cell.SCORING : Cell.AUDIENCE;
  }

  /**
   * The wiggle in a Flower, this long after the drive in was done: {heading offset from facing it, its rate}, radians and rad/s,
   * CCW positive. Starts at 0 turning left, so the heading doesn't jump.
   */
  /** Seconds since this step's drive was done, or NaN while it is still driving. */
  public double arrivedForS(double nowS) {
    return nowS - arrivedAtS;
  }

  public static double[] flowerWiggle(double inStepS) {
    double w = 2 * Math.PI / WIGGLE_PERIOD_S;
    return new double[] {WIGGLE_RAD * Math.sin(w * inStepS), WIGGLE_RAD * w * Math.cos(w * inStepS)};
  }
}
