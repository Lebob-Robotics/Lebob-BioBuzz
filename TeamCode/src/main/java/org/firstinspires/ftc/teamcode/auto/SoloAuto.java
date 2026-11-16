package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.Units;
import org.psilynx.psikit.core.Logger;

/**
 * The solo autonomous ({@link SoloSequence}), for one alliance, driven as {@link BaseAuto} drives: the preloads from
 * beside our Garden, the Garden, a check of the Hive, then two Flowers and a park whose volleys depend on whether
 * the Hive tipped. With no partner, also the partner's preloads from the loading zone, shot from the park.
 */
public abstract class SoloAuto extends BaseAuto<SoloSequence.Step, SoloSequence> {
  /**
   * The drive to a Flower hands over to lowering the arm this far from the stop short of it, facing it within
   * FACE_FLOWER_RAD: the arm comes down (AutoSequence.ARM_MOVE_S) as the robot covers the rest, not after it stops.
   */
  public static double LOWER_ARM_FROM_M = 0.25;
  /**
   * A drive this long or longer goes on one of the robot's axes ({@link #turning}): forward, back or sideways, never
   * diagonally, where a mecanum drive is slowest. At full power in the regionals logs: 1.5 m/s forward, 1.3
   * strafing, 1.1 diagonally.
   */
  public static double ALONG_MIN_M = 0.8;
  /** Along such a drive, as fractions of it: on the axis by the first, turning on from the second, done by the third. */
  public static double ALONG_ON_AXIS_BY = 0.10;
  public static double ALONG_TURN_FROM = 0.80;
  public static double ALONG_END_BY = 0.90;

  private final boolean partnerBalls;
  private TipPredictor tips;
  private Cell checkUpAtStart;
  private int checkFramesAtStart;
  private Boolean tagsTipped;

  protected SoloAuto(Alliance alliance) {
    this(alliance, false);
  }

  /** @param partnerBalls no partner: its preloads are taken from the loading zone ({@link SoloSequence}) */
  protected SoloAuto(Alliance alliance, boolean partnerBalls) {
    super(alliance);
    this.partnerBalls = partnerBalls;
  }

  @Override
  protected double[] redStart() {
    return SoloSequence.START;
  }

  @Override
  protected SoloSequence newPlan() {
    tips = new TipPredictor(robot.getUpCell());
    return new SoloSequence(0, partnerBalls);
  }

  @Override
  protected String placement() {
    String wall = alliance == Alliance.RED ? "audience" : "rear";
    return alliance + " SOLO: robot against the " + wall + " wall beside our Garden, its side 2 cm past the Garden's"
        + " tile, intake to the wall, camera to the Hive"
        + (partnerBalls ? ". NO PARTNER: its preloads at the loading zone's middle against our wall" : "");
  }

  @Override
  protected String status(double t) {
    Boolean tipped = plan.tipped();
    return String.format(Locale.US, "SOLO%s %s  %.1f s  %d shot  aim %s  %s", partnerBalls ? " NO PARTNER" : "",
        plan.step(), t, plan.ballsShot(),
        aimCell(), tipped == null ? "" : tipped ? "TIPPED" : "NOT TIPPED");
  }

  @Override
  protected void onLoopStart(double t) {
    tips.onVision(robot.getUpCell(), robot.getUpCellAgreeingFrames(), t);
    if (plan.watchingHive()) {
      Cell up = robot.getUpCell();
      if (checkUpAtStart == null) { // Only frames from the watch's start count: the Hive may have tipped just before
        checkUpAtStart = up;
        checkFramesAtStart = robot.getUpCellAgreeingFrames();
      }
      boolean fresh = up != checkUpAtStart
          || robot.getUpCellAgreeingFrames() - checkFramesAtStart >= SoloSequence.TIP_CHECK_FRAMES;
      tagsTipped = fresh ? up != SoloSequence.nearCell(alliance) : null;
    } else {
      checkUpAtStart = null;
      tagsTipped = null;
    }
    plan.tipCheck(tagsTipped, tips.cell() != SoloSequence.nearCell(alliance));
  }

  @Override
  protected void onBallLeft(double t) {
    tips.onBall(t);
  }

  @Override
  protected void onWiggle(double headingRad) {
    Logger.recordOutput("Auto/HeadingTarget", headingRad);
  }

  @Override
  protected void logExtras(double t) {
    Boolean tipped = plan.tipped();
    Logger.recordOutput("Auto/PredictedCell", tips.cell().name());
    Logger.recordOutput("Auto/SoloTipped", tipped == null ? "" : tipped ? "TIPPED" : "NOT_TIPPED");
    Logger.recordOutput("Auto/SoloTipFromTags", plan.tipFromTags());
    Boolean farTipped = plan.farTipped();
    Logger.recordOutput("Auto/SoloFarTipped", farTipped == null ? "" : farTipped ? "TIPPED" : "NOT_TIPPED");
    Logger.recordOutput("Auto/SoloFarTipFromTags", plan.farTipFromTags());
    if (partnerBalls) {
      Logger.recordOutput("Auto/SoloParkVolleyCell", plan.parkVolleyCell(alliance).name());
    }
  }

  /**
   * The step's drive is done, as {@link ScoringAuto#stepArrived}: pushed into a Flower or the Garden's corner, on a
   * facing step's spot at its heading, near enough a Flower's stop to lower the arm, else Pedro's path finished.
   */
  @Override
  protected boolean stepArrived(SoloSequence.Step step, boolean facingStep, double inStepS, double robotSpeedMps) {
    if (step == SoloSequence.Step.DRIVE_TO_WALL_FLOWER || step == SoloSequence.Step.DRIVE_TO_REAR_FLOWER) {
      double[] stop = AutoField.forAlliance(
          step == SoloSequence.Step.DRIVE_TO_WALL_FLOWER ? AutoField.WALL_FLOWER : AutoField.REAR_FLOWER, alliance);
      FieldPose pose = pose();
      boolean armRoom = Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= LOWER_ARM_FROM_M
          && Math.abs(Angles.normalize(pose.headingRad - stop[2])) <= FACE_FLOWER_RAD;
      return armRoom || !follower.following();
    }
    if ((step == SoloSequence.Step.INTAKE_WALL_FLOWER || step == SoloSequence.Step.INTAKE_REAR_FLOWER
        || step == SoloSequence.Step.TAKE_PARTNER_BALLS)
        && inStepS >= FLOWER_STALL_AFTER_S && robotSpeedMps <= FLOWER_STALLED_MPS) {
      return true;
    }
    if (step == SoloSequence.Step.SWEEP_GARDEN && inStepS >= FLOWER_STALL_AFTER_S
        && robotSpeedMps <= ScoringAuto.WALL_STOPPED_MPS) {
      return true;
    }
    if (!facingStep) {
      return !follower.following();
    }
    if (step == SoloSequence.Step.ONTO_GARDEN_WALL && inStepS >= ScoringAuto.WALL_SETTLE_S
        && robotSpeedMps <= ScoringAuto.WALL_STOPPED_MPS) {
      return true;
    }
    double[] stop = AutoField.forAlliance(holdSpot(step), alliance);
    FieldPose pose = pose();
    double within = step == SoloSequence.Step.ONTO_GARDEN_WALL ? ScoringAuto.ONTO_WALL_M : FACE_FLOWER_M;
    return Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= within
        && Math.abs(Angles.normalize(pose.headingRad - stop[2])) <= FACE_FLOWER_RAD;
  }

  /** The Cell of the next volley, so the hood and wheels are set on the way. */
  @Override
  protected Cell aimCell() {
    SoloSequence.Step volley = plan == null ? SoloSequence.Step.SHOOT_PRELOAD : plan.nextVolley();
    if (volley == SoloSequence.Step.SHOOT_PARK) {
      return plan.parkVolleyCell(alliance);
    }
    return volley == SoloSequence.Step.SHOOT_PRELOAD || volley == SoloSequence.Step.SHOOT_NEAR
        ? SoloSequence.nearCell(alliance) : SoloSequence.parkCell(alliance);
  }

  @Override
  protected double flowerHeading(SoloSequence.Step step) {
    return step == SoloSequence.Step.INTAKE_WALL_FLOWER || step == SoloSequence.Step.INTAKE_REAR_FLOWER
        || step == SoloSequence.Step.TAKE_PARTNER_BALLS ? lockedHeading(step) : Double.NaN;
  }

  @Override
  protected void startStep(SoloSequence.Step step) {
    FieldPose now = pose();
    Pose here = new Pose(now.xM / Units.M_PER_IN, now.yM / Units.M_PER_IN, now.headingRad);
    boolean slow = step == SoloSequence.Step.SWEEP_GARDEN || step == SoloSequence.Step.INTAKE_WALL_FLOWER
        || step == SoloSequence.Step.INTAKE_REAR_FLOWER || step == SoloSequence.Step.TAKE_PARTNER_BALLS;
    drivetrain.setScale(slow ? SWEEP_POWER_SCALE : 1.0);
    boolean onWall = step == SoloSequence.Step.ONTO_GARDEN_WALL || step == SoloSequence.Step.SWEEP_GARDEN;
    drivetrain.setStrafeBias(onWall ? ScoringAuto.WALL_PUSH : 0.0);
    drivetrain.setForwardBias(0.0);
    lockHeading(lockedHeading(step));
    Pose park = poseOf(SoloSequence.parkSpot(alliance));
    switch (step) {
      case SHOOT_PRELOAD: // Off the wall during the spin-up, turning to the near Cell on the way
        line(here, poseOf(SoloSequence.preloadSpot(alliance)), true);
        break;
      case FACE_GARDEN: // Intake first along the Pollen line, clear of the wall to turn
        follower.hold(pedro(AutoField.GARDEN_IN, alliance));
        break;
      case ONTO_GARDEN_WALL: // Sideways onto the audience wall, not turning
        follower.hold(pedro(AutoField.GARDEN_WALL, alliance));
        break;
      case SWEEP_GARDEN: { // Along the wall, against it, from the line's start: Pedro closes any gap left
        Pose end = pedro(AutoField.gardenSweepEnd(), alliance);
        follower.follow(Paths.line(pedro(AutoField.GARDEN_WALL, alliance), end).constant(end.heading()));
        break;
      }
      case CHECK_TIP: // Out of the corner, cameras to the Hive, to where the not-tipped volley is shot
        line(here, AutoField.pedroShotSpot(alliance, SoloSequence.nearCell(alliance)), true);
        break;
      case SHOOT_NEAR:
        lineTo(here, AutoField.pedroShotSpot(alliance, SoloSequence.nearCell(alliance)));
        break;
      case DRIVE_TO_PARK:
        toPark(here, now, park);
        break;
      case SHOOT_FAR:
        lineTo(here, AutoField.pedroShotSpot(alliance, SoloSequence.parkCell(alliance)));
        break;
      case DRIVE_TO_WALL_FLOWER:
        lineTo(here, pedro(AutoField.WALL_FLOWER, alliance));
        break;
      case LOWER_ARM_WALL: // The rest of the way to the stop, facing the Flower, the arm going down
        follower.hold(pedro(AutoField.WALL_FLOWER, alliance));
        break;
      case INTAKE_WALL_FLOWER: // Arm down, straight in still facing it
        line(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_WALL_FLOWER: // Arm still down, straight back out to the stop; it comes up on the next drive
        line(here, pedro(AutoField.WALL_FLOWER, alliance), false);
        break;
      case DRIVE_TO_REAR_FLOWER: // From the park or the near spot's side, up beside the Hive
        lineTo(here, pedro(AutoField.REAR_FLOWER, alliance));
        break;
      case LOWER_ARM_REAR:
        follower.hold(pedro(AutoField.REAR_FLOWER, alliance));
        break;
      case INTAKE_REAR_FLOWER:
        line(here, pedro(AutoField.REAR_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_REAR_FLOWER:
        line(here, pedro(AutoField.REAR_FLOWER, alliance), false);
        break;
      case DRIVE_TO_PARTNER_BALLS: // Square to our wall above them, turning on the way
        lineTo(here, pedro(AutoField.PARTNER_BALLS_IN, alliance));
        break;
      case TAKE_PARTNER_BALLS: // Straight into the wall through them, wiggling as in a Flower
        line(here, pedro(AutoField.partnerBallsEnd(), alliance), false);
        break;
      case SHOOT_PARK: // Back off the wall to the park, turning to the Cell
        lineTo(here, poseOf(plan.parkedSpot(alliance)));
        break;
      case PARKED: // Held off our wall: square to it, or where the park volley was shot
        follower.hold(poseOf(plan.parkedSpot(alliance)));
        break;
    }
  }

  /** To the park: straight, or bent over our wall's Flower where straight crosses it. */
  private void toPark(Pose here, FieldPose now, Pose park) {
    double[] control = SoloSequence.parkControl(
        AutoField.forAlliance(new double[] {now.xM, now.yM, now.headingRad}, alliance));
    if (control != null && here.distance(park) >= MIN_PATH_IN) {
      follower.follow(turning(Paths.curve(here, pedro(control, alliance), park), here, park));
    } else {
      lineTo(here, park);
    }
  }

  /** Straight to the end, turning to its heading on the way as {@link #turning} does. */
  private void lineTo(Pose here, Pose end) {
    if (here.distance(end) < MIN_PATH_IN) {
      follower.hold(end);
      return;
    }
    follower.follow(turning(Paths.line(here, end), here, end));
  }

  /**
   * The path's headings from here's to the end's: evenly on a short drive. From {@link #ALONG_MIN_M} on, it turns
   * early onto the quickest of the robot's axes along the drive ({@link SoloSequence#alongAxis}), holds it, and is at
   * the end's heading before it arrives, so the volley or the Flower's arm does not wait for the turn.
   */
  private static Path turning(Path path, Pose here, Pose end) {
    if (here.distance(end) * Units.M_PER_IN < ALONG_MIN_M) {
      return path.linear(here.heading(), end.heading());
    }
    double axis = SoloSequence.alongAxis(here.heading(), end.heading(),
        Math.atan2(end.y() - here.y(), end.x() - here.x()), here.distance(end) * Units.M_PER_IN);
    PiecewiseInterpolator headings = Interpolator.piecewise()
        .until(ALONG_ON_AXIS_BY, Interpolator.linear(here.heading(), axis))
        .until(ALONG_TURN_FROM, Interpolator.constant(axis))
        .until(ALONG_END_BY, Interpolator.linear(axis, end.heading()));
    if (ALONG_END_BY < 1) {
      headings.until(1, Interpolator.constant(end.heading()));
    }
    return path.heading(headings);
  }

  /** The heading the lock holds taking the Garden or a Flower, else NaN to leave the turn to Pedro. */
  private double lockedHeading(SoloSequence.Step step) {
    switch (step) {
      case FACE_GARDEN:
      case ONTO_GARDEN_WALL:
      case SWEEP_GARDEN:
        return AutoField.forAlliance(AutoField.GARDEN_IN, alliance)[2];
      case LOWER_ARM_WALL:
      case INTAKE_WALL_FLOWER:
      case BACK_FROM_WALL_FLOWER:
        return AutoField.forAlliance(AutoField.WALL_FLOWER, alliance)[2];
      case LOWER_ARM_REAR:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_REAR_FLOWER:
        return AutoField.forAlliance(AutoField.REAR_FLOWER, alliance)[2];
      case TAKE_PARTNER_BALLS:
        return AutoField.forAlliance(AutoField.PARTNER_BALLS_IN, alliance)[2];
      default:
        return Double.NaN;
    }
  }

  /** Where a facing step (red) holds. */
  private static double[] holdSpot(SoloSequence.Step step) {
    switch (step) {
      case FACE_GARDEN:
        return AutoField.GARDEN_IN;
      case ONTO_GARDEN_WALL:
        return AutoField.GARDEN_WALL;
      case LOWER_ARM_WALL:
        return AutoField.WALL_FLOWER;
      default:
        return AutoField.REAR_FLOWER;
    }
  }

  private static Pose poseOf(double[] field) {
    return new Pose(field[0] / Units.M_PER_IN, field[1] / Units.M_PER_IN, field[2]);
  }
}
