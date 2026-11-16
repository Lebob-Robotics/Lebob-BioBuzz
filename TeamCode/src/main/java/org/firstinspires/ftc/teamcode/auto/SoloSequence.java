package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * The solo autonomous's steps and when each ends, without hardware, and where it goes (red field coordinates,
 * metres, as {@link AutoField}). Made to score alone from beside our Garden:
 *
 * <ol>
 *   <li>Start against the audience wall beside our Garden, camera side to the Hive. Pull off the wall to Volley
 *       Tuning's spot 11 while the flywheels spin up, and shoot the 4 preloads into the Cell beside the Garden (the
 *       one up at the start: {@link #nearCell}).
 *   <li>Take the Garden as {@link AutoSequence} does: turn intake first along the Pollen line, slide onto the wall,
 *       sweep into the corner.
 *   <li>Check the Hive on the way out of the corner, cameras to it, driving to the spot in front of the near Cell.
 *       The tags decide as soon as they show it; with none by the spot, the ball count's prediction does.
 *   <li>Not tipped: shoot the Garden there, take our wall's Flower, shoot from the park, take the rear Flower, then
 *       park and shoot what it holds until the end.
 *   <li>Tipped: drive on to the park and shoot the Garden into the other Cell from there, take the rear Flower and
 *       shoot from in front of that Cell, take our wall's Flower and shoot into the near Cell, then park and shoot
 *       until the end in case a ball stuck.
 * </ol>
 *
 * <p>A Flower is not started after {@link #LATEST_FLOWER_S}: it goes to park instead. Whatever it is doing at
 * {@link #PARK_BY_S}, it goes to park; in a Flower it starts backing out {@link AutoSequence#FLOWER_EXIT_S} before
 * that, and from a Flower left that late it parks rather than drive to another volley. Flower, Garden and volley timings are {@link AutoSequence}'s.
 */
public final class SoloSequence implements AutoPlan<SoloSequence.Step> {
  public enum Step {
    SHOOT_PRELOAD(true),
    FACE_GARDEN(false), ONTO_GARDEN_WALL(false), SWEEP_GARDEN(false),
    CHECK_TIP(false),
    /** In front of the near Cell, at {@link AutoField#shotSpot}. */
    SHOOT_NEAR(true),
    /** From the park, at the other Cell. */
    SHOOT_PARK(true),
    /** In front of the other Cell, at {@link AutoField#shotSpot}. */
    SHOOT_FAR(true),
    DRIVE_TO_WALL_FLOWER(false), LOWER_ARM_WALL(false), INTAKE_WALL_FLOWER(false), BACK_FROM_WALL_FLOWER(false),
    RAISE_ARM_WALL(false),
    DRIVE_TO_REAR_FLOWER(false), LOWER_ARM_REAR(false), INTAKE_REAR_FLOWER(false), BACK_FROM_REAR_FLOWER(false),
    RAISE_ARM_REAR(false),
    DRIVE_TO_PARK(false), PARKED(true);

    /** A volley is shot here, once the robot has stopped at the step's spot. */
    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  private static final List<Step> OPENING = Arrays.asList(
      Step.SHOOT_PRELOAD, Step.FACE_GARDEN, Step.ONTO_GARDEN_WALL, Step.SWEEP_GARDEN, Step.CHECK_TIP);
  private static final List<Step> WALL_FLOWER = Arrays.asList(Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL,
      Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER, Step.RAISE_ARM_WALL);
  private static final List<Step> REAR_FLOWER = Arrays.asList(Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR,
      Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER, Step.RAISE_ARM_REAR);

  /** Latest start of the drive to park: about 2.2 s from the near Cell's spot, parked before AUTO ends at 30 s. */
  public static double PARK_BY_S = 26.0;
  /** No Flower is started after this: its drive, the arm and the dwell would not leave time for a volley. */
  public static double LATEST_FLOWER_S = 21.0;
  /** Fresh tag frames that settle the tip check: frames agreeing with the up Cell, or a flip, since it began. */
  public static int TIP_CHECK_FRAMES = 3;
  /** At the near Cell's spot with no fresh frames, this long more before the ball count's prediction decides. */
  public static double TIP_CHECK_WAIT_S = 0.3;
  /** Longest for the check whatever the drive does. */
  public static double TIP_CHECK_MAX_S = 2.5;

  /** Red start: against the audience wall, its side 2 cm past the Garden's tile, intake to the wall. */
  public static final double[] START = {AutoField.AT_WALL_M, -(AutoField.FIELD_HALF_M - 0.61) + 0.02
      + AutoField.ROBOT_HALF_M, 0};
  /**
   * Where the preloads are shot from (red, {x, y}): Volley Tuning's spot 11 for the near Cell, 1.11 m from its aim
   * point, 0.3 m off the audience wall: clear of it to turn, and 0.12 m from where the Garden's turn is held.
   */
  public static final double[] PRELOAD_SPOT = {1.30, -0.95};
  /**
   * Bends the drive from the Garden's corner to the park over our wall's Flower: straight, it crossed the Flower's
   * keep-out (AutoField.WALL_FLOWER_KEEP_OUT).
   */
  public static final double[] PARK_ENTRY_CONTROL = {1.00, -0.95, 0};

  private final List<Step> route = new ArrayList<>(OPENING);
  private int at = 0;
  private Step step = Step.SHOOT_PRELOAD;
  private final StepTracker tracker;
  private boolean parking = false;
  private Boolean tipped = null;
  private boolean tipFromTags = false;
  private Boolean tagsSayTipped = null;
  private boolean predictedTipped = false;

  public SoloSequence(double startS) {
    tracker = new StepTracker(startS);
  }

  /** The Cell beside our Garden, up at the start: red AUDIENCE, blue SCORING. */
  public static Cell nearCell(Alliance alliance) {
    double startX = AutoField.forAlliance(START, alliance)[0];
    for (Cell c : Cell.values()) {
      if (Math.signum(AutoField.autoAimPoint(alliance, c)[0]) == Math.signum(startX)) {
        return c;
      }
    }
    throw new IllegalStateException("No Cell on the start's side");
  }

  /** The other Cell, beside the loading zone: the park's. */
  public static Cell parkCell(Alliance alliance) {
    return nearCell(alliance).other();
  }

  /** Where the preloads are shot from, in field coordinates for the alliance, back (shooter) to the near Cell. */
  public static double[] preloadSpot(Alliance alliance) {
    double[] p = AutoField.forAlliance(new double[] {PRELOAD_SPOT[0], PRELOAD_SPOT[1], 0}, alliance);
    double[] aim = AutoField.autoAimPoint(alliance, nearCell(alliance));
    return new double[] {p[0], p[1], Math.atan2(p[1] - aim[1], p[0] - aim[0])};
  }

  /** Where it parks and shoots from, for the alliance: AutoField's park, back to {@link #parkCell}. */
  public static double[] parkSpot(Alliance alliance) {
    return AutoField.parkSpot(alliance, parkCell(alliance));
  }

  /**
   * The tip check's inputs, each loop before {@link #update}.
   *
   * @param tagsTipped the near Cell is down by the tags' fresh frames; null while they have shown nothing new
   * @param predicted  the near Cell is down by the ball count (TipPredictor), for when the tags show nothing
   */
  public void tipCheck(Boolean tagsTipped, boolean predicted) {
    tagsSayTipped = tagsTipped;
    predictedTipped = predicted;
  }

  @Override
  public void update(double nowS, boolean pathDone, boolean ballLeft, boolean feederEmpty, boolean stackFull) {
    tracker.countBalls(nowS, ballLeft, feederEmpty);
    if (inFlower() && nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) {
      leaveFlower(nowS);
      return;
    }
    if (!parking && nowS >= PARK_BY_S && !leavingFlower()) {
      goPark(nowS);
      return;
    }
    tracker.arrive(nowS, pathDone, step.shooting);
    double inStepS = tracker.inStepS(nowS);
    switch (step) {
      case SHOOT_PRELOAD:
      case SHOOT_NEAR:
      case SHOOT_PARK:
      case SHOOT_FAR:
        if (tracker.volleyEnded(nowS)) {
          next(nowS);
        }
        break;
      case FACE_GARDEN: // pathDone here: clear of the wall and facing along the Pollen line
        if (pathDone || inStepS >= AutoSequence.FACE_GARDEN_MAX_S) {
          next(nowS);
        }
        break;
      case ONTO_GARDEN_WALL: // pathDone here: against the wall at the line's start
        if (pathDone || inStepS >= AutoSequence.ONTO_WALL_MAX_S) {
          next(nowS);
        }
        break;
      case SWEEP_GARDEN:
        if (pathDone || inStepS >= AutoSequence.GARDEN_SWEEP_MAX_S) {
          next(nowS);
        }
        break;
      case CHECK_TIP:
        checkTip(nowS, inStepS);
        break;
      case DRIVE_TO_WALL_FLOWER:
      case DRIVE_TO_REAR_FLOWER:
        if (pathDone) {
          next(nowS);
        }
        break;
      case LOWER_ARM_WALL:
      case LOWER_ARM_REAR:
        // pathDone here: stopped short of the Flower and facing it. The arm goes down then
        if ((tracker.arrived() && tracker.arrivedForS(nowS) >= AutoSequence.ARM_MOVE_S)
            || inStepS >= AutoSequence.FACE_FLOWER_MAX_S) {
          next(nowS);
        }
        break;
      case INTAKE_WALL_FLOWER:
      case INTAKE_REAR_FLOWER:
        boolean dwelt = tracker.arrived() && tracker.arrivedForS(nowS) >= AutoSequence.FLOWER_DWELL_S;
        if (stackFull || dwelt || inStepS >= AutoSequence.FLOWER_INTAKE_S) {
          leaveFlower(nowS);
        }
        break;
      case BACK_FROM_WALL_FLOWER:
      case BACK_FROM_REAR_FLOWER:
        if (pathDone || inStepS >= AutoSequence.BACK_OUT_MAX_S) {
          next(nowS);
        }
        break;
      case RAISE_ARM_WALL:
      case RAISE_ARM_REAR:
        if (inStepS >= AutoSequence.ARM_MOVE_S) {
          if (nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) { // No time for a volley elsewhere: the park's
            goPark(nowS);
          } else {
            next(nowS);
          }
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

  /** Decides once the tags show the Hive, or with the prediction once at the spot with nothing from them. */
  private void checkTip(double nowS, double inStepS) {
    Boolean verdict = tagsSayTipped;
    tipFromTags = verdict != null;
    boolean waited = tracker.arrived() && tracker.arrivedForS(nowS) >= TIP_CHECK_WAIT_S;
    if (verdict == null && (waited || inStepS >= TIP_CHECK_MAX_S)) {
      verdict = predictedTipped;
    }
    if (verdict == null) {
      return;
    }
    tipped = verdict;
    if (tipped) {
      route.add(Step.SHOOT_PARK);
      route.addAll(REAR_FLOWER);
      route.add(Step.SHOOT_FAR);
      route.addAll(WALL_FLOWER);
      route.add(Step.SHOOT_NEAR);
    } else {
      route.add(Step.SHOOT_NEAR);
      route.addAll(WALL_FLOWER);
      route.add(Step.SHOOT_PARK);
      route.addAll(REAR_FLOWER);
    }
    route.add(Step.DRIVE_TO_PARK);
    route.add(Step.PARKED);
    next(nowS);
  }

  /** On to the route's next step; a Flower too late to start goes to park instead. */
  private void next(double nowS) {
    at++;
    Step following = route.get(at);
    if ((following == Step.DRIVE_TO_WALL_FLOWER || following == Step.DRIVE_TO_REAR_FLOWER)
        && nowS >= LATEST_FLOWER_S) {
      goPark(nowS);
      return;
    }
    enter(following, nowS);
  }

  /** To park, or parked already when shooting from it. */
  private void goPark(double nowS) {
    enter(step == Step.SHOOT_PARK || step == Step.PARKED ? Step.PARKED : Step.DRIVE_TO_PARK, nowS);
  }

  private boolean inFlower() {
    return step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER
        || step == Step.LOWER_ARM_REAR || step == Step.INTAKE_REAR_FLOWER;
  }

  /** Out of a Flower: backs out if the arm is down, else straight on to what follows the Flower. */
  private void leaveFlower(double nowS) {
    boolean wall = step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER;
    if (armDown()) {
      at = route.indexOf(wall ? Step.BACK_FROM_WALL_FLOWER : Step.BACK_FROM_REAR_FLOWER);
      enter(route.get(at), nowS);
      return;
    }
    at = route.indexOf(wall ? Step.RAISE_ARM_WALL : Step.RAISE_ARM_REAR);
    if (nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) {
      goPark(nowS);
    } else {
      next(nowS);
    }
  }

  private boolean leavingFlower() {
    return step == Step.BACK_FROM_WALL_FLOWER || step == Step.RAISE_ARM_WALL
        || step == Step.BACK_FROM_REAR_FLOWER || step == Step.RAISE_ARM_REAR;
  }

  private void enter(Step following, double nowS) {
    parking |= following == Step.DRIVE_TO_PARK || following == Step.PARKED;
    step = following;
    tracker.enter(nowS, following.shooting);
  }

  @Override
  public Step step() {
    return step;
  }

  @Override
  public boolean takeNewStep() {
    return tracker.takeNewStep();
  }

  /** Shooting: stopped at a shooting spot, its volley not finished. Parked, until the end. */
  @Override
  public boolean shoot(double nowS) {
    return step.shooting && tracker.arrived() && (step == Step.PARKED || !tracker.volleyDone(nowS));
  }

  /**
   * The intake runs taking the Garden and the Flowers, through each volley after a pickup and the check's drive
   * (the Garden's balls sat in the intake's mouth with it off), and once stopped for the preloads and parked.
   */
  @Override
  public boolean intake() {
    switch (step) {
      case SHOOT_PRELOAD:
      case PARKED:
        return tracker.arrived();
      case DRIVE_TO_WALL_FLOWER:
      case DRIVE_TO_REAR_FLOWER:
      case DRIVE_TO_PARK:
        return false;
      default:
        return true;
    }
  }

  /** True while the indexer stops in its agitation: taking the Garden or a Flower, in each cycle's stop part. */
  @Override
  public boolean indexerPaused(double nowS) {
    switch (step) {
      case FACE_GARDEN:
      case ONTO_GARDEN_WALL:
      case SWEEP_GARDEN:
      case LOWER_ARM_WALL:
      case INTAKE_WALL_FLOWER:
      case LOWER_ARM_REAR:
      case INTAKE_REAR_FLOWER:
        return AutoSequence.agitationPaused(tracker.inStepS(nowS), AutoSequence.AGITATE_RUN_S,
            AutoSequence.AGITATE_STOP_S);
      default:
        return false;
    }
  }

  /** The Pollen arm is down from facing a Flower until it backs out, as {@link AutoSequence#armDown}. */
  @Override
  public boolean armDown() {
    switch (step) {
      case LOWER_ARM_WALL:
      case LOWER_ARM_REAR:
        return tracker.arrived();
      case INTAKE_WALL_FLOWER:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_WALL_FLOWER:
      case BACK_FROM_REAR_FLOWER:
        return true;
      default:
        return false;
    }
  }

  @Override
  public boolean isFacingStep() {
    return step == Step.FACE_GARDEN || step == Step.ONTO_GARDEN_WALL || step == Step.LOWER_ARM_WALL
        || step == Step.LOWER_ARM_REAR;
  }

  @Override
  public int ballsShot() {
    return tracker.ballsShot();
  }

  @Override
  public double arrivedForS(double nowS) {
    return tracker.arrivedForS(nowS);
  }

  /** The tip check's answer: null until it is made. */
  public Boolean tipped() {
    return tipped;
  }

  /** The check was settled by the tags, not the prediction. */
  public boolean tipFromTags() {
    return tipFromTags;
  }

  /**
   * The next volley from here: this step's if it shoots, else the first shooting step after it; before the check is
   * made, the one its likely answer leads to.
   */
  public Step nextVolley() {
    if (step == Step.SHOOT_PRELOAD) {
      return step;
    }
    if (tipped == null) {
      boolean likely = tagsSayTipped != null ? tagsSayTipped : predictedTipped;
      return likely ? Step.SHOOT_PARK : Step.SHOOT_NEAR;
    }
    if (parking) {
      return Step.PARKED;
    }
    for (int i = at; i < route.size(); i++) {
      if (route.get(i).shooting) {
        return route.get(i);
      }
    }
    return Step.PARKED;
  }
}
