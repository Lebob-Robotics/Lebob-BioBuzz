package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;

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
 * the Hive tipped.
 */
public abstract class SoloAuto extends BaseAuto<SoloSequence.Step, SoloSequence> {
  private TipPredictor tips;
  private Cell checkUpAtStart;
  private int checkFramesAtStart;
  private Boolean tagsTipped;

  protected SoloAuto(Alliance alliance) {
    super(alliance);
  }

  @Override
  protected double[] redStart() {
    return SoloSequence.START;
  }

  @Override
  protected SoloSequence newPlan() {
    tips = new TipPredictor(robot.getUpCell());
    return new SoloSequence(0);
  }

  @Override
  protected String placement() {
    String wall = alliance == Alliance.RED ? "audience" : "rear";
    return alliance + " SOLO: robot against the " + wall + " wall beside our Garden, its side 2 cm past the Garden's"
        + " tile, intake to the wall, camera to the Hive";
  }

  @Override
  protected String status(double t) {
    Boolean tipped = plan.tipped();
    return String.format(Locale.US, "SOLO %s  %.1f s  %d shot  aim %s  %s", plan.step(), t, plan.ballsShot(),
        aimCell(), tipped == null ? "" : tipped ? "TIPPED" : "NOT TIPPED");
  }

  @Override
  protected void onLoopStart(double t) {
    tips.onVision(robot.getUpCell(), robot.getUpCellAgreeingFrames(), t);
    if (plan.step() == SoloSequence.Step.CHECK_TIP) {
      Cell up = robot.getUpCell();
      if (checkUpAtStart == null) { // Only frames from the check on count: the Hive may tip after the preloads
        checkUpAtStart = up;
        checkFramesAtStart = robot.getUpCellAgreeingFrames();
      }
      boolean fresh = up != checkUpAtStart
          || robot.getUpCellAgreeingFrames() - checkFramesAtStart >= SoloSequence.TIP_CHECK_FRAMES;
      tagsTipped = fresh ? up != SoloSequence.nearCell(alliance) : null;
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
  }

  /**
   * The step's drive is done, as {@link ScoringAuto#stepArrived}: pushed into a Flower or the Garden's corner, on a
   * facing step's spot at its heading, else Pedro's path finished.
   */
  @Override
  protected boolean stepArrived(SoloSequence.Step step, boolean facingStep, double inStepS, double robotSpeedMps) {
    if ((step == SoloSequence.Step.INTAKE_WALL_FLOWER || step == SoloSequence.Step.INTAKE_REAR_FLOWER)
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
    return volley == SoloSequence.Step.SHOOT_PRELOAD || volley == SoloSequence.Step.SHOOT_NEAR
        ? SoloSequence.nearCell(alliance) : SoloSequence.parkCell(alliance);
  }

  @Override
  protected double flowerHeading(SoloSequence.Step step) {
    return step == SoloSequence.Step.INTAKE_WALL_FLOWER || step == SoloSequence.Step.INTAKE_REAR_FLOWER
        ? lockedHeading(step) : Double.NaN;
  }

  @Override
  protected void startStep(SoloSequence.Step step) {
    FieldPose now = pose();
    Pose here = new Pose(now.xM / Units.M_PER_IN, now.yM / Units.M_PER_IN, now.headingRad);
    boolean slow = step == SoloSequence.Step.SWEEP_GARDEN || step == SoloSequence.Step.INTAKE_WALL_FLOWER
        || step == SoloSequence.Step.INTAKE_REAR_FLOWER;
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
      case SHOOT_NEAR:
        line(here, AutoField.pedroShotSpot(alliance, SoloSequence.nearCell(alliance)), true);
        break;
      case SHOOT_PARK:
      case DRIVE_TO_PARK:
        toPark(here, now, park);
        break;
      case SHOOT_FAR:
        line(here, AutoField.pedroShotSpot(alliance, SoloSequence.parkCell(alliance)), true);
        break;
      case DRIVE_TO_WALL_FLOWER:
        line(here, pedro(AutoField.WALL_FLOWER, alliance), true);
        break;
      case LOWER_ARM_WALL: // Faces the Flower where the drive stopped; the arm goes down once it does
        follower.hold(pedro(AutoField.WALL_FLOWER, alliance));
        break;
      case INTAKE_WALL_FLOWER: // Arm down, straight in still facing it
        line(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_WALL_FLOWER: // Arm still down, straight back out to the stop
        line(here, pedro(AutoField.WALL_FLOWER, alliance), false);
        break;
      case DRIVE_TO_REAR_FLOWER: // From the park or the near spot's side, up beside the Hive
        line(here, pedro(AutoField.REAR_FLOWER, alliance), true);
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
      case RAISE_ARM_WALL: // Still while the arm comes up
      case RAISE_ARM_REAR:
        follower.hold(here);
        break;
      case PARKED: // Held off our wall, back to the park's Cell, shooting until the end
        follower.hold(park);
        break;
    }
  }

  /** To the park, back (shooter) to its Cell; from the Garden's side of our wall's Flower, bent over it. */
  private void toPark(Pose here, FieldPose now, Pose park) {
    double[] red = AutoField.forAlliance(new double[] {now.xM, now.yM, now.headingRad}, alliance);
    if (red[0] > AutoField.WALL_FLOWER_KEEP_OUT[1] && here.distance(park) >= MIN_PATH_IN) {
      follower.follow(Paths.curve(here, pedro(SoloSequence.PARK_ENTRY_CONTROL, alliance), park)
          .linear(here.heading(), park.heading()));
    } else {
      line(here, park, true);
    }
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
      case RAISE_ARM_WALL:
        return AutoField.forAlliance(AutoField.WALL_FLOWER, alliance)[2];
      case LOWER_ARM_REAR:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_REAR_FLOWER:
      case RAISE_ARM_REAR:
        return AutoField.forAlliance(AutoField.REAR_FLOWER, alliance)[2];
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
