package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * The solo autonomous's steps and when each ends, without hardware, and where it goes (red field coordinates,
 * metres, as {@link AutoField}). Made to score alone from beside our Garden:
 *
 * <ol>
 *   <li>Start against the audience wall beside our Garden, camera side to the Hive. Drive to Volley Tuning's spot 5
 *       for the Cell beside the Garden (the one up at the start: {@link #nearCell}) while the flywheels spin up, and
 *       shoot the 4 preloads into it.
 *   <li>Take the Garden as {@link AutoSequence} does: turn intake first along the Pollen line, slide onto the wall,
 *       sweep into the corner.
 *   <li>Check the Hive on the way out of the corner, cameras to it, driving to the spot in front of the near Cell.
 *       The tags decide as soon as they show it; with none by the spot, the ball count's prediction does.
 *   <li>Not tipped: shoot the Garden there, take our wall's Flower, shoot from in front of the other Cell, take the
 *       rear Flower, shoot from in front of the other Cell again, then park.
 *   <li>Tipped: drive on to the spot in front of the other Cell and shoot the Garden into it, take the rear Flower and
 *       shoot from there again, then take our wall's Flower. Watching the Hive from there (camera to it), recheck:
 *       once the other Cell has tipped back (the near Cell up again), shoot into the near Cell, then park. Not tipped
 *       back, shoot the other Cell again, to tip it, then park.
 * </ol>
 *
 * <p>It parks at {@link AutoField#EDGE_PARK}, beside the loading zone and just over its line, leaving the zone to a
 * partner. Parked, it shoots nothing: square to our wall it does not face a Cell.
 *
 * <p>With no partner ({@link #SoloSequence(double, boolean)}), the not-tipped route ends instead by taking the
 * partner's 4 preloads, put at the loading zone's middle against our wall, intake first into the wall through them,
 * cameras to the Hive. It shoots them from Volley Tuning's PARK spot into whichever Cell is up by those frames, else
 * the ball count, and stays there, parked. The tipped route has no time for them.
 *
 * <p>At a Flower the robot never stops for the arm: it goes down on the last of the drive to the stop short of the
 * Flower ({@link SoloAuto#LOWER_ARM_FROM_M}), and comes up on the drive on from the stop after backing out.
 *
 * <p>A Flower is not started after {@link #LATEST_FLOWER_S}: it goes to park instead. Whatever it is doing at
 * {@link #PARK_BY_S} ({@link #PARK_FROM_NEAR_BY_S} at the near Cell's volley), it goes to park; in a Flower it starts
 * backing out {@link AutoSequence#FLOWER_EXIT_S} before that, and from a Flower left that late it parks rather than
 * drive to another volley. Flower, Garden and volley
 * timings are {@link AutoSequence}'s.
 *
 * <p>Every volley is shot from a spot Volley Tuning confirmed, so the auto shoots with that spot's settings: spot 5
 * in front of either Cell (93% IN at the 15 Nov competition).
 */
public final class SoloSequence implements AutoPlan<SoloSequence.Step> {
  public enum Step {
    SHOOT_PRELOAD(true),
    FACE_GARDEN(false), ONTO_GARDEN_WALL(false), SWEEP_GARDEN(false),
    CHECK_TIP(false),
    /** In front of the near Cell, at {@link AutoField#shotSpot}. */
    SHOOT_NEAR(true),
    /** In front of the other Cell, beside the park, at {@link AutoField#shotSpot}. */
    SHOOT_FAR(true),
    DRIVE_TO_WALL_FLOWER(false), LOWER_ARM_WALL(false), INTAKE_WALL_FLOWER(false), BACK_FROM_WALL_FLOWER(false),
    DRIVE_TO_REAR_FLOWER(false), LOWER_ARM_REAR(false), INTAKE_REAR_FLOWER(false), BACK_FROM_REAR_FLOWER(false),
    DRIVE_TO_PARK(false), PARKED(false),
    /** No partner: square to our wall above its preloads, then intake first into the wall through them. */
    DRIVE_TO_PARTNER_BALLS(false), TAKE_PARTNER_BALLS(false),
    /** No partner: at Volley Tuning's PARK spot ({@link AutoField#parkSpot}), into the Cell that is up. */
    SHOOT_PARK(true);

    /** A volley is shot here, once the robot has stopped at the step's spot. */
    public final boolean shooting;

    Step(boolean shooting) {
      this.shooting = shooting;
    }
  }

  private static final List<Step> OPENING = Arrays.asList(
      Step.SHOOT_PRELOAD, Step.FACE_GARDEN, Step.ONTO_GARDEN_WALL, Step.SWEEP_GARDEN, Step.CHECK_TIP);
  private static final List<Step> WALL_FLOWER = Arrays.asList(Step.DRIVE_TO_WALL_FLOWER, Step.LOWER_ARM_WALL,
      Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER);
  private static final List<Step> REAR_FLOWER = Arrays.asList(Step.DRIVE_TO_REAR_FLOWER, Step.LOWER_ARM_REAR,
      Step.INTAKE_REAR_FLOWER, Step.BACK_FROM_REAR_FLOWER);

  /** Latest start of the drive to park, parked before AUTO ends at 30 s: about 1 s from the far Cell's spot. */
  public static double PARK_BY_S = 26.0;
  /** From the near Cell's spot, the farthest from the park (about 2.5 s), the drive starts by this instead. */
  public static double PARK_FROM_NEAR_BY_S = 25.7;
  /** No Flower is started after this: its drive, the arm and the dwell would not leave time for a volley. */
  public static double LATEST_FLOWER_S = 21.0;
  /**
   * The partner's preloads are not started after this. Started by it, the robot is at the park by about 27.3 s
   * (SoloAutoSimTest); started at 23.5 s, as usual, the last of them leaves at 28.0 s.
   */
  public static double LATEST_PARTNER_BALLS_S = 24.5;
  /** Stopped against our wall in the partner's preloads this long, unless the stack fills first. */
  public static double PARTNER_DWELL_S = 0.5;
  /** Longest for the drive into the wall through them. */
  public static double PARTNER_TAKE_MAX_S = 2.0;
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
   * Bends the drive from the Garden's corner to the park over our wall's Flower: straight, it crossed the Flower's
   * keep-out (AutoField.WALL_FLOWER_KEEP_OUT). From where the tags usually decide the check, it passes 9 cm outside
   * the keep-out and 29 cm from the Hive. Lower bends were up to 0.1 s quicker in SoloAutoSimTest, but within 3 cm.
   */
  public static final double[] PARK_ENTRY_CONTROL = {1.40, -1.05, 0};

  private final boolean partnerBalls;
  private final List<Step> route = new ArrayList<>(OPENING);
  private int at = 0;
  private Step step = Step.SHOOT_PRELOAD;
  private final StepTracker tracker;
  private boolean parking = false;
  private Boolean tipped = null;
  private boolean tipFromTags = false;
  private Boolean tagsSayTipped = null;
  private boolean predictedTipped = false;
  private Boolean farTipped = null;
  private boolean farTipFromTags = false;
  private Boolean parkVolleyNear = null;
  private boolean lastVolleyEmptied = false;

  public SoloSequence(double startS) {
    this(startS, false);
  }

  /** @param partnerBalls no partner: its preloads are taken from the loading zone and shot from the park */
  public SoloSequence(double startS, boolean partnerBalls) {
    tracker = new StepTracker(startS);
    this.partnerBalls = partnerBalls;
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

  /** The other Cell, on the side of the loading zone's rear end, where it parks. */
  public static Cell parkCell(Alliance alliance) {
    return nearCell(alliance).other();
  }

  /**
   * Where the preloads are shot from, back (shooter) to the near Cell: Volley Tuning's spot 5 for it, as the Garden's
   * volley when the Hive has not tipped. Spot 7 is 0.15 m nearer the start at 90%, but its tuned indexer runs at 0.75
   * and feeds slower; spot 11, beside the start, scored 73%.
   */
  public static double[] preloadSpot(Alliance alliance) {
    return AutoField.shotSpot(alliance, nearCell(alliance));
  }

  /** Where it parks, for the alliance: {@link AutoField#EDGE_PARK}. */
  public static double[] parkSpot(Alliance alliance) {
    return AutoField.edgePark(alliance);
  }

  /**
   * The bend for the drive to the park from here (red coordinates), or null for a straight one: bent over our wall's
   * Flower only where the straight drive would cross its keep-out.
   */
  public static double[] parkControl(double[] redFrom) {
    return crossesWallFlower(redFrom, parkSpot(Alliance.RED)) ? PARK_ENTRY_CONTROL : null;
  }

  /** Full-power speeds driving along the robot (forward or back) and across it, from the regionals logs. */
  public static double ALONG_MPS = 1.5;
  public static double ACROSS_MPS = 1.2;
  /** Turning on the way costs about this much time per radian beyond the turn from start to end heading. */
  public static double TURN_COST_S_PER_RAD = 0.4;

  /**
   * The heading to drive travelRad on one of the robot's axes (forward, back or either side), never diagonally where
   * a mecanum drive is slowest: whichever is quickest for the distance with the turning it adds, onto it from the
   * start's heading and from it on to the end's.
   */
  static double alongAxis(double fromRad, double toRad, double travelRad, double distanceM) {
    double direct = Math.abs(Angles.normalize(toRad - fromRad));
    double best = Angles.normalize(travelRad);
    double quickest = Double.MAX_VALUE;
    for (int k = 0; k < 4; k++) {
      double axis = Angles.normalize(travelRad + k * Math.PI / 2);
      double turn = Math.abs(Angles.normalize(axis - fromRad)) + Math.abs(Angles.normalize(toRad - axis));
      double s = distanceM / (k % 2 == 0 ? ALONG_MPS : ACROSS_MPS) + (turn - direct) * TURN_COST_S_PER_RAD;
      if (s < quickest) {
        quickest = s;
        best = axis;
      }
    }
    return best;
  }

  /** The straight drive between two red points takes the robot's centre into our wall's Flower's keep-out. */
  static boolean crossesWallFlower(double[] redFrom, double[] redTo) {
    double[] keepOut = AutoField.WALL_FLOWER_KEEP_OUT;
    for (int i = 0; i <= 100; i++) {
      double x = redFrom[0] + (redTo[0] - redFrom[0]) * i / 100.0;
      double y = redFrom[1] + (redTo[1] - redFrom[1]) * i / 100.0;
      if (x > keepOut[0] && x < keepOut[1] && y < keepOut[2]) {
        return true;
      }
    }
    return false;
  }

  /**
   * The tip check's and the recheck's inputs, each loop before {@link #update}.
   *
   * @param tagsTipped the near Cell is down by the tags' frames since {@link #watchingHive} began; null while they
   *                   have shown nothing new
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
    if (!parking && nowS >= parkByS() && !leavingFlower()) {
      goPark(nowS);
      return;
    }
    tracker.arrive(nowS, pathDone, step.shooting);
    double inStepS = tracker.inStepS(nowS);
    switch (step) {
      case SHOOT_PRELOAD:
      case SHOOT_NEAR:
      case SHOOT_FAR:
      case SHOOT_PARK:
        if (tracker.volleyEnded(nowS)) {
          lastVolleyEmptied = tracker.volleyDone(nowS);
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
        // The arm goes down from the step's start, on the last of the drive; pathDone here: at the stop facing the
        // Flower
        if ((tracker.arrived() && inStepS >= AutoSequence.ARM_MOVE_S) || inStepS >= AutoSequence.FACE_FLOWER_MAX_S) {
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
      case BACK_FROM_REAR_FLOWER: // Backed out clear of the Flower: the arm comes up on the next drive
        if (pathDone || inStepS >= AutoSequence.BACK_OUT_MAX_S) {
          if (nowS >= PARK_BY_S - AutoSequence.FLOWER_EXIT_S) { // No time for another volley
            goPark(nowS);
          } else if (step == Step.BACK_FROM_WALL_FLOWER && recheckPending()) {
            recheckFarTip(nowS);
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
      case DRIVE_TO_PARTNER_BALLS:
        if (pathDone) {
          next(nowS);
        }
        break;
      case TAKE_PARTNER_BALLS: // pathDone here: stopped against our wall
        boolean taken = tracker.arrived() && tracker.arrivedForS(nowS) >= PARTNER_DWELL_S;
        if (stackFull || taken || inStepS >= PARTNER_TAKE_MAX_S) {
          next(nowS);
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
      route.add(Step.SHOOT_FAR);
      route.addAll(REAR_FLOWER);
      route.add(Step.SHOOT_FAR);
      route.addAll(WALL_FLOWER);
      route.add(Step.SHOOT_NEAR);
    } else {
      route.add(Step.SHOOT_NEAR);
      route.addAll(WALL_FLOWER);
      route.add(Step.SHOOT_FAR);
      route.addAll(REAR_FLOWER);
      route.add(Step.SHOOT_FAR);
    }
    if (!tipped && partnerBalls) {
      route.addAll(Arrays.asList(Step.DRIVE_TO_PARTNER_BALLS, Step.TAKE_PARTNER_BALLS, Step.SHOOT_PARK));
    } else {
      route.add(Step.DRIVE_TO_PARK);
    }
    route.add(Step.PARKED);
    next(nowS);
  }

  /**
   * The near Cell's volley only once the other Cell has tipped back: by the tags' frames at our wall's Flower, else
   * the ball count. Not tipped back, the other Cell is shot again, then it parks.
   */
  private void recheckFarTip(double nowS) {
    farTipFromTags = tagsSayTipped != null;
    farTipped = !(farTipFromTags ? tagsSayTipped : predictedTipped);
    if (!farTipped) {
      route.subList(at + 1, route.size()).clear();
      route.add(Step.SHOOT_FAR);
      route.add(Step.DRIVE_TO_PARK);
      route.add(Step.PARKED);
    }
    next(nowS);
  }

  /** Tipped, the near Cell's volley still ahead and not yet rechecked. */
  private boolean recheckPending() {
    return Boolean.TRUE.equals(tipped) && farTipped == null && route.indexOf(Step.SHOOT_NEAR) > at;
  }

  /**
   * The tags' frames count toward a check now: during the tip check, and for the recheck at our wall's Flower, where
   * the camera faces the Hive and the other Cell's last volley has landed. Each watch counts only its own frames.
   */
  public boolean watchingHive() {
    boolean atWallFlower = step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER
        || step == Step.BACK_FROM_WALL_FLOWER;
    return step == Step.CHECK_TIP || (atWallFlower && recheckPending()) || step == Step.TAKE_PARTNER_BALLS;
  }

  /**
   * On to the route's next step; a Flower too late to start goes to park instead, as do the partner's preloads too
   * late or after a volley given up with balls still in: 4 more would be over the 4 a robot may control (G407).
   */
  private void next(double nowS) {
    at++;
    Step following = route.get(at);
    boolean lateFlower = (following == Step.DRIVE_TO_WALL_FLOWER || following == Step.DRIVE_TO_REAR_FLOWER)
        && nowS >= LATEST_FLOWER_S;
    boolean noPartnerBalls = following == Step.DRIVE_TO_PARTNER_BALLS
        && (nowS >= LATEST_PARTNER_BALLS_S || !lastVolleyEmptied);
    if (lateFlower || noPartnerBalls) {
      goPark(nowS);
      return;
    }
    enter(following, nowS);
  }

  private double parkByS() {
    return step == Step.SHOOT_NEAR ? PARK_FROM_NEAR_BY_S : PARK_BY_S;
  }

  /** To park; at the partner's preloads, already at the park, it shoots what it took from there. */
  private void goPark(double nowS) {
    if (step == Step.DRIVE_TO_PARTNER_BALLS || step == Step.TAKE_PARTNER_BALLS) {
      at = route.indexOf(Step.SHOOT_PARK);
      enter(Step.SHOOT_PARK, nowS);
      return;
    }
    enter(Step.DRIVE_TO_PARK, nowS);
  }

  /** The near Cell is down: by the tags' frames of the current watch, else the ball count. */
  private boolean nearDown() {
    return tagsSayTipped != null ? tagsSayTipped : predictedTipped;
  }

  private boolean inFlower() {
    return step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER
        || step == Step.LOWER_ARM_REAR || step == Step.INTAKE_REAR_FLOWER;
  }

  /** Out of a Flower: the arm is down from the drive's last part on, so it backs out. */
  private void leaveFlower(double nowS) {
    boolean wall = step == Step.LOWER_ARM_WALL || step == Step.INTAKE_WALL_FLOWER;
    at = route.indexOf(wall ? Step.BACK_FROM_WALL_FLOWER : Step.BACK_FROM_REAR_FLOWER);
    enter(route.get(at), nowS);
  }

  private boolean leavingFlower() {
    return step == Step.BACK_FROM_WALL_FLOWER || step == Step.BACK_FROM_REAR_FLOWER;
  }

  private void enter(Step following, double nowS) {
    if (following == Step.SHOOT_PARK) {
      parkVolleyNear = !nearDown();
    }
    parking |= following == Step.DRIVE_TO_PARK || following == Step.PARKED || following == Step.SHOOT_PARK;
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

  /** Shooting: stopped at a shooting spot, its volley not finished. */
  @Override
  public boolean shoot(double nowS) {
    return step.shooting && tracker.arrived() && !tracker.volleyDone(nowS);
  }

  /**
   * The intake runs taking the Garden and the Flowers, through each volley after a pickup and the check's drive
   * (the Garden's balls sat in the intake's mouth with it off), and once stopped for the preloads.
   */
  @Override
  public boolean intake() {
    switch (step) {
      case SHOOT_PRELOAD:
        return tracker.arrived();
      case DRIVE_TO_WALL_FLOWER:
      case DRIVE_TO_REAR_FLOWER:
      case DRIVE_TO_PARTNER_BALLS:
      case DRIVE_TO_PARK:
      case PARKED:
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
      case TAKE_PARTNER_BALLS:
        return AutoSequence.agitationPaused(tracker.inStepS(nowS), AutoSequence.AGITATE_RUN_S,
            AutoSequence.AGITATE_STOP_S);
      default:
        return false;
    }
  }

  /** The Pollen arm is down from the last of the drive to a Flower until it has backed out. */
  @Override
  public boolean armDown() {
    switch (step) {
      case LOWER_ARM_WALL:
      case LOWER_ARM_REAR:
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

  /** The recheck's answer, tipped: the other Cell tipped back, so the near Cell is shot. Null until it is made. */
  public Boolean farTipped() {
    return farTipped;
  }

  /** The recheck was settled by the tags, not the prediction. */
  public boolean farTipFromTags() {
    return farTipFromTags;
  }

  /** The park volley's Cell, the one up: decided on leaving the partner's preloads, the likely one before. */
  public Cell parkVolleyCell(Alliance alliance) {
    boolean near = parkVolleyNear != null ? parkVolleyNear : !nearDown();
    return near ? nearCell(alliance) : parkCell(alliance);
  }

  /** Where it ends, for the alliance: {@link #parkSpot}, or after a park volley the spot it was shot from. */
  public double[] parkedSpot(Alliance alliance) {
    return parkVolleyNear == null ? parkSpot(alliance) : AutoField.parkSpot(alliance, parkVolleyCell(alliance));
  }

  /**
   * The next volley from here: this step's if it shoots, else the first shooting step after it; before the check or
   * the recheck is made, the one its likely answer leads to.
   */
  public Step nextVolley() {
    if (step == Step.SHOOT_PRELOAD || step == Step.SHOOT_PARK) {
      return step;
    }
    if (tipped == null) {
      boolean likely = tagsSayTipped != null ? tagsSayTipped : predictedTipped;
      return likely ? Step.SHOOT_FAR : Step.SHOOT_NEAR;
    }
    if (parking) {
      return Step.PARKED;
    }
    for (int i = at; i < route.size(); i++) {
      if (route.get(i) == Step.SHOOT_NEAR && recheckPending()) {
        boolean nearDown = tagsSayTipped != null ? tagsSayTipped : predictedTipped;
        return nearDown ? Step.SHOOT_FAR : Step.SHOOT_NEAR;
      }
      if (route.get(i).shooting) {
        return route.get(i);
      }
    }
    return Step.PARKED;
  }
}
