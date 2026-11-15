package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;

/**
 * The corner autonomous's steps and when each ends, without hardware, and where it goes (red field coordinates,
 * metres, as {@link AutoField}). Made to fit around a partner whose auto takes the Garden side (15 Nov):
 *
 * <ol>
 *   <li>Start in the rear corner of our wall, beside the loading zone, intake to our wall. Drive {@link
 *       #PRELOAD_OUT_M} toward the near Cell, turn to it there and shoot the 4 preloads.
 *   <li>Square back up to our wall, back into the corner, and wait there {@link #WAIT_S}.
 *   <li>Drive along our wall, just off it, to our wall's Flower (between the loading zone and the Garden) and take it
 *       with the Pollen arm, as {@link AutoSequence} does.
 *   <li>Shoot into the other Cell, AUDIENCE, from the spot in front of it (Volley Tuning's spot there).
 *   <li>Drive back and park at the loading zone's end nearest the Garden.
 * </ol>
 *
 * <p>Whatever it is doing at {@link #PARK_BY_S}, it goes to park; in the Flower it starts backing out
 * {@link AutoSequence#FLOWER_EXIT_S} before that, so the arm is up by then. Flower and volley timings are {@link
 * AutoSequence}'s.
 */
public final class CornerSequence {
  public enum Step {
    SHOOT_PRELOAD(true), FACE_WALL(false), BACK_TO_CORNER(false), WAIT_IN_CORNER(false),
    DRIVE_TO_FLOWER(false), LOWER_ARM(false), INTAKE_FLOWER(false), BACK_FROM_FLOWER(false), RAISE_ARM(false),
    SHOOT_GARDEN_SIDE(true),
    DRIVE_TO_PARK(false), PARKED(false);

    /** A volley is shot here, once the robot has stopped at the step's spot. */
    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  /** Back in the corner, this long before leaving for the Flower (15 Nov). */
  public static double WAIT_S = 5.0;
  /** Leaves the corner by this time however late it got back, for the Flower and the second volley to fit. */
  public static double LEAVE_BY_S = 17.0;
  /** Latest start of the drive to park: about 1.9 m from the AUDIENCE spot. */
  public static double PARK_BY_S = 26.0;
  /** Longest to square up to our wall after the preloads, before backing into the corner anyway. */
  public static double FACE_WALL_MAX_S = 1.5;
  /** Longest to back into the corner. */
  public static double BACK_TO_CORNER_MAX_S = 2.5;

  /** From the corner, this far toward the near Cell to shoot the preloads: there it turns clear of both walls. */
  public static double PRELOAD_OUT_M = 0.20;
  /**
   * Taking the Flower and on to the AUDIENCE spot, the indexer loads JIGGLE_RUN_S then runs backward
   * JIGGLE_REVERSE_S, and over: the volley retry's pulse, so the stack indexes up (15 Nov: the volley after the
   * Flower fed 2 s and shot none). 0 reverse is off.
   */
  public static double JIGGLE_RUN_S = 0.4;
  public static double JIGGLE_REVERSE_S = 0.15;
  /** The drive along our wall keeps this far off it: 5 cm was too close (15 Nov). */
  public static double EDGE_GAP_M = 0.10;

  /** The rear corner of our wall, touching it and the rear wall, intake to our wall: the start. */
  public static final double[] CORNER = {-AutoField.AT_WALL_M, -AutoField.AT_WALL_M, -Math.PI / 2};
  /** Waiting in the corner, just off both walls. */
  public static final double[] CORNER_WAIT = {CORNER[0] + 0.02, CORNER[1] + 0.02, CORNER[2]};
  /** The drive along our wall ends here, its robot clear of the Flower, and bends up to the Flower's stop. */
  public static final double[] EDGE_END = {-0.05, -(AutoField.AT_WALL_M - EDGE_GAP_M), -Math.PI / 2};
  public static final double[] FLOWER_BEND = {0.0, AutoField.WALL_FLOWER[1], 0};
  /**
   * Parked at the loading zone's end nearest the Garden, its centre 0.16 m inside the zone's line (the robot over it
   * by 7 cm), {@link AutoField#PARK_WALL_GAP_M} off our wall, intake to it. 3 cm inside went a little too far
   * (15 Nov).
   */
  public static final double[] CORNER_PARK = {AutoField.LOADING_ZONE[1] - AutoField.ROBOT_HALF_M + 0.07,
      -(AutoField.AT_WALL_M - AutoField.PARK_WALL_GAP_M), -Math.PI / 2};

  private Step step = Step.SHOOT_PRELOAD;
  private double stepStartS;
  private double arrivedAtS = Double.NaN;
  private double emptySinceS = Double.NaN;
  private int ballsThisVolley = 0;
  private int ballsShot = 0;
  private boolean newStep = true;

  public CornerSequence(double startS) {
    stepStartS = startS;
  }

  /** The preloads' Cell: SCORING, both alliances (15 Nov). */
  public static Cell nearCell(Alliance alliance) {
    return Cell.SCORING;
  }

  /** The second volley's Cell, after the Flower: AUDIENCE, both alliances (15 Nov). */
  public static Cell gardenCell(Alliance alliance) {
    return Cell.AUDIENCE;
  }


  /**
   * Where the preloads are shot from, in field coordinates for the alliance: {@link #PRELOAD_OUT_M} from the corner
   * toward the near Cell's aim point, still facing our wall (the aim turns it there).
   */
  public static double[] preloadSpot(Alliance alliance) {
    double[] corner = AutoField.forAlliance(CORNER, alliance);
    double[] aim = AutoField.autoAimPoint(alliance, nearCell(alliance));
    double dx = aim[0] - corner[0];
    double dy = aim[1] - corner[1];
    double d = Math.hypot(dx, dy);
    return new double[] {corner[0] + PRELOAD_OUT_M * dx / d, corner[1] + PRELOAD_OUT_M * dy / d, corner[2]};
  }

  /**
   * One loop.
   *
   * @param pathDone    the step's drive has reached its spot (for a step that turns in place, and faces its way)
   * @param ballLeft    a ball left the shooter this loop
   * @param feederEmpty the feed gate found no ball on its last pulse
   * @param stackFull   the robot is full: the Flower ends at once
   */
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
    if (inFlower() && nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) {
      leaveFlower(nowS);
      return;
    }
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
    boolean late = nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S; // No time for the Flower
    switch (step) {
      case SHOOT_PRELOAD:
        if (volleyEnded(nowS)) {
          enter(Step.FACE_WALL, nowS);
        }
        break;
      case FACE_WALL:
        if (pathDone || inStepS >= FACE_WALL_MAX_S) {
          enter(Step.BACK_TO_CORNER, nowS);
        }
        break;
      case BACK_TO_CORNER:
        if (pathDone || inStepS >= BACK_TO_CORNER_MAX_S) {
          enter(Step.WAIT_IN_CORNER, nowS);
        }
        break;
      case WAIT_IN_CORNER:
        if (inStepS >= WAIT_S || nowS >= LEAVE_BY_S) {
          enter(Step.DRIVE_TO_FLOWER, nowS);
        }
        break;
      case DRIVE_TO_FLOWER:
        if (pathDone) {
          enter(late ? Step.SHOOT_GARDEN_SIDE : Step.LOWER_ARM, nowS);
        }
        break;
      case LOWER_ARM: // pathDone here: stopped short of the Flower and facing it. The arm goes down then
        if ((!Double.isNaN(arrivedAtS) && nowS - arrivedAtS >= AutoSequence.ARM_MOVE_S)
            || inStepS >= AutoSequence.FACE_FLOWER_MAX_S) {
          enter(Step.INTAKE_FLOWER, nowS);
        }
        break;
      case INTAKE_FLOWER:
        boolean dwelt = !Double.isNaN(arrivedAtS) && nowS - arrivedAtS >= AutoSequence.FLOWER_DWELL_S;
        if (stackFull || dwelt || inStepS >= AutoSequence.FLOWER_INTAKE_S) {
          leaveFlower(nowS);
        }
        break;
      case BACK_FROM_FLOWER:
        if (pathDone || inStepS >= AutoSequence.BACK_OUT_MAX_S) {
          enter(Step.RAISE_ARM, nowS);
        }
        break;
      case RAISE_ARM:
        if (inStepS >= AutoSequence.ARM_MOVE_S) {
          enter(Step.SHOOT_GARDEN_SIDE, nowS);
        }
        break;
      case SHOOT_GARDEN_SIDE:
        if (volleyEnded(nowS)) {
          enter(Step.DRIVE_TO_PARK, nowS);
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
    return step == Step.LOWER_ARM || step == Step.INTAKE_FLOWER;
  }

  /** Out of the Flower: backs out if the arm is down, else straight on to the volley. */
  private void leaveFlower(double nowS) {
    enter(armDown() ? Step.BACK_FROM_FLOWER : Step.SHOOT_GARDEN_SIDE, nowS);
  }

  private boolean leavingFlower() {
    return step == Step.BACK_FROM_FLOWER || step == Step.RAISE_ARM;
  }

  /** At the shooting spot, and the volley finished or given up on. */
  private boolean volleyEnded(double nowS) {
    if (Double.isNaN(arrivedAtS)) {
      return false;
    }
    return volleyDone(nowS) || nowS - arrivedAtS >= AutoSequence.VOLLEY_WAIT_S;
  }

  private boolean volleyDone(double nowS) {
    return ballsThisVolley >= AutoSequence.BALLS_PER_VOLLEY
        || (!Double.isNaN(emptySinceS) && nowS - emptySinceS >= AutoSequence.EMPTY_S);
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

  /** Shooting: stopped at a shooting spot, its volley not finished. */
  public boolean shoot(double nowS) {
    return step.shooting && !Double.isNaN(arrivedAtS) && !volleyDone(nowS);
  }

  /** The intake runs through each volley once stopped (the AUDIENCE one's from the Flower on) and at the Flower. */
  public boolean intake() {
    switch (step) {
      case SHOOT_PRELOAD: // Once stopped only, for the 4th ball in the intake (14 Nov 220630)
        return !Double.isNaN(arrivedAtS);
      case LOWER_ARM:
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
      case RAISE_ARM:
      case SHOOT_GARDEN_SIDE: // The whole volley, drive too: the Flower's balls sit in the intake's mouth (14 Nov)
        return true;
      default:
        return false;
    }
  }

  /**
   * True in the jiggle's backward part: from lowering the arm at the Flower until stopped at the AUDIENCE spot, where
   * the volley's own retry takes over.
   */
  public boolean indexerPaused(double nowS) {
    switch (step) {
      case LOWER_ARM:
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
      case RAISE_ARM:
      case SHOOT_GARDEN_SIDE:
        if (!Double.isNaN(arrivedAtS)) {
          return false;
        }
        break;
      default:
        return false;
    }
    return AutoSequence.agitationPaused(nowS - stepStartS, JIGGLE_RUN_S, JIGGLE_REVERSE_S);
  }

  /** The Pollen arm is down: from facing the Flower until it backs out. */
  public boolean armDown() {
    switch (step) {
      case LOWER_ARM:
        return !Double.isNaN(arrivedAtS);
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
        return true;
      default:
        return false;
    }
  }

  /** Holding a spot until the robot is on it and at its heading: squared to our wall, or facing the Flower. */
  public boolean isFacingStep() {
    return step == Step.FACE_WALL || step == Step.LOWER_ARM;
  }

  public int ballsShot() {
    return ballsShot;
  }

  /** Seconds since this step's drive was done, or NaN while it is still driving. */
  public double arrivedForS(double nowS) {
    return nowS - arrivedAtS;
  }
}
