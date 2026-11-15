package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * The corner autonomous's steps and when each ends, without hardware, and where it goes (red field coordinates,
 * metres, as {@link AutoField}). Made to fit around a partner whose auto takes the Garden side:
 *
 * <ol>
 *   <li>Start in the rear corner of our wall, beside the loading zone, intake to our wall. Drive out to Volley
 *       Tuning's spot 5 in front of SCORING ({@link AutoField#shotSpot}), turn to it there and shoot the 4 preloads.
 *   <li>Square back up to our wall, back into the corner, and wait there {@link #WAIT_S}.
 *   <li>Drive along our wall, just off it, to our wall's Flower (between the loading zone and the Garden) and take it
 *       with the Pollen arm, as {@link AutoSequence} does.
 *   <li>Shoot into the other Cell, AUDIENCE, from spot 5 in front of it.
 *   <li>Drive back and park at the loading zone's end nearest the Garden, back to {@link #parkCell}, and shoot
 *       whatever is left.
 * </ol>
 *
 * <p>Whatever it is doing at {@link #PARK_BY_S}, it goes to park; in the Flower it starts backing out
 * {@link AutoSequence#FLOWER_EXIT_S} before that, so the arm is up by then. Flower and volley timings are {@link
 * AutoSequence}'s.
 */
public final class CornerSequence implements AutoPlan<CornerSequence.Step> {
  public enum Step {
    SHOOT_PRELOAD(true), FACE_WALL(false), BACK_TO_CORNER(false), WAIT_IN_CORNER(false),
    DRIVE_TO_FLOWER(false), LOWER_ARM(false), INTAKE_FLOWER(false), BACK_FROM_FLOWER(false), RAISE_ARM(false),
    SHOOT_GARDEN_SIDE(true),
    DRIVE_TO_PARK(false), PARKED(true);

    /** A volley is shot here, once the robot has stopped at the step's spot. */
    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  /** Back in the corner, this long before leaving for the Flower. */
  public static double WAIT_S = 5.0;
  /** Leaves the corner by this time however late it got back, for the Flower and the second volley to fit. */
  public static double LEAVE_BY_S = 17.0;
  /** Latest start of the drive to park: about 1.9 m from the AUDIENCE spot. */
  public static double PARK_BY_S = 26.0;
  /** Longest to square up to our wall after the preloads, before backing into the corner anyway. */
  public static double FACE_WALL_MAX_S = 1.5;
  /** Longest to back into the corner. */
  public static double BACK_TO_CORNER_MAX_S = 2.5;

  /**
   * Taking the Flower and on to the AUDIENCE spot, the indexer loads JIGGLE_RUN_S then runs backward
   * JIGGLE_REVERSE_S, and over: the volley retry's pulse, so the stack indexes up (without it the volley after the
   * Flower shot none). 0 reverse is off.
   */
  public static double JIGGLE_RUN_S = 0.4;
  public static double JIGGLE_REVERSE_S = 0.15;
  /** The drive along our wall keeps this far off it. */
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
   * by 7 cm), {@link AutoField#PARK_WALL_GAP_M} off our wall, intake to it. Its x is the park's; {@link #parkSpot}
   * turns it to shoot.
   */
  public static final double[] CORNER_PARK = {AutoField.LOADING_ZONE[1] - AutoField.ROBOT_HALF_M + 0.07,
      -(AutoField.AT_WALL_M - AutoField.PARK_WALL_GAP_M), -Math.PI / 2};

  private Step step = Step.SHOOT_PRELOAD;
  private final StepTracker tracker;

  public CornerSequence(double startS) {
    tracker = new StepTracker(startS);
  }

  /** The preloads' Cell: SCORING, both alliances. */
  public static Cell nearCell(Alliance alliance) {
    return Cell.SCORING;
  }

  /** The second volley's Cell, after the Flower: AUDIENCE, both alliances. */
  public static Cell gardenCell(Alliance alliance) {
    return Cell.AUDIENCE;
  }

  /** The leftovers' Cell, shot parked: SCORING, the nearer, and the one Volley Tuning's PARK spot is for. */
  public static Cell parkCell(Alliance alliance) {
    return Cell.SCORING;
  }

  /**
   * Where it parks, for the alliance: {@link #CORNER_PARK}'s x, back (shooter) to {@link #parkCell}, the turned
   * robot's nearest corner {@link AutoField#PARK_WALL_GAP_M} off our wall.
   */
  public static double[] parkSpot(Alliance alliance) {
    return AutoField.parkSpot(alliance, parkCell(alliance), CORNER_PARK);
  }


  /**
   * Where the preloads are shot from, in field coordinates for the alliance: Volley Tuning's spot 5 in front of the
   * preloads' Cell. Still facing our wall: the drive doesn't turn out of the corner, the aim turns it there.
   */
  public static double[] preloadSpot(Alliance alliance) {
    double[] spot = AutoField.shotSpot(alliance, nearCell(alliance));
    return new double[] {spot[0], spot[1], AutoField.forAlliance(CORNER, alliance)[2]};
  }

  /**
   * One loop.
   *
   * @param pathDone    the step's drive has reached its spot (for a step that turns in place, and faces its way)
   * @param ballLeft    a ball left the shooter this loop
   * @param feederEmpty the feed gate found no ball on its last pulse
   * @param stackFull   the robot is full: the Flower ends at once
   */
  @Override
  public void update(double nowS, boolean pathDone, boolean ballLeft, boolean feederEmpty, boolean stackFull) {
    tracker.countBalls(nowS, ballLeft, feederEmpty);
    if (inFlower() && nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) {
      leaveFlower(nowS);
      return;
    }
    if (step.ordinal() < Step.DRIVE_TO_PARK.ordinal() && nowS >= PARK_BY_S && !leavingFlower()) {
      enter(Step.DRIVE_TO_PARK, nowS);
      return;
    }
    tracker.arrive(nowS, pathDone, step.shooting);
    double inStepS = tracker.inStepS(nowS);
    boolean late = nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S; // No time for the Flower
    switch (step) {
      case SHOOT_PRELOAD:
        if (tracker.volleyEnded(nowS)) {
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
        if ((tracker.arrived() && tracker.arrivedForS(nowS) >= AutoSequence.ARM_MOVE_S)
            || inStepS >= AutoSequence.FACE_FLOWER_MAX_S) {
          enter(Step.INTAKE_FLOWER, nowS);
        }
        break;
      case INTAKE_FLOWER:
        boolean dwelt = tracker.arrived() && tracker.arrivedForS(nowS) >= AutoSequence.FLOWER_DWELL_S;
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
        if (tracker.volleyEnded(nowS)) {
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

  private void enter(Step next, double nowS) {
    step = next;
    tracker.enter(nowS, next.shooting);
  }

  @Override
  public Step step() {
    return step;
  }

  @Override
  public boolean takeNewStep() {
    return tracker.takeNewStep();
  }

  /** Shooting: stopped at a shooting spot, its volley not finished. */
  @Override
  public boolean shoot(double nowS) {
    return step.shooting && tracker.arrived() && !tracker.volleyDone(nowS);
  }

  /** The intake runs through each volley once stopped (the AUDIENCE one's from the Flower on) and at the Flower. */
  @Override
  public boolean intake() {
    switch (step) {
      case SHOOT_PRELOAD: // Once stopped only, for the 4th ball in the intake
        return tracker.arrived();
      case LOWER_ARM:
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
      case RAISE_ARM:
      case SHOOT_GARDEN_SIDE: // The whole volley, drive too: the Flower's balls sit in the intake's mouth
      case PARKED:
        return true;
      default:
        return false;
    }
  }

  /**
   * True in the jiggle's backward part: from lowering the arm at the Flower until stopped at the AUDIENCE spot, where
   * the volley's own retry takes over.
   */
  @Override
  public boolean indexerPaused(double nowS) {
    switch (step) {
      case LOWER_ARM:
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
      case RAISE_ARM:
      case SHOOT_GARDEN_SIDE:
        if (tracker.arrived()) {
          return false;
        }
        break;
      default:
        return false;
    }
    return AutoSequence.agitationPaused(tracker.inStepS(nowS), JIGGLE_RUN_S, JIGGLE_REVERSE_S);
  }

  /** The Pollen arm is down: from facing the Flower until it backs out. */
  @Override
  public boolean armDown() {
    switch (step) {
      case LOWER_ARM:
        return tracker.arrived();
      case INTAKE_FLOWER:
      case BACK_FROM_FLOWER:
        return true;
      default:
        return false;
    }
  }

  /** Holding a spot until the robot is on it and at its heading: squared to our wall, or facing the Flower. */
  @Override
  public boolean isFacingStep() {
    return step == Step.FACE_WALL || step == Step.LOWER_ARM;
  }

  @Override
  public int ballsShot() {
    return tracker.ballsShot();
  }

  @Override
  public double arrivedForS(double nowS) {
    return tracker.arrivedForS(nowS);
  }
}
