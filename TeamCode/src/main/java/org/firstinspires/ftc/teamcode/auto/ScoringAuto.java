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
 * The shooting autonomous ({@link AutoSequence}), for one alliance, driven as {@link BaseAuto} drives: the preloads
 * into the Cell up at the start, the Garden into the other, our wall's Flower, then park.
 */
public abstract class ScoringAuto extends BaseAuto<AutoSequence.Step, AutoSequence> {
  /** Onto the Garden's wall is closer: the sweep runs against it. */
  public static double ONTO_WALL_M = 0.02;
  /**
   * Strafe power added toward the audience wall (the robot's left, both alliances) while sliding onto it and sweeping
   * along it. 14 Nov Pedro alone stalled 5.8 cm off at 0.14 strafe, and swept 2.5-3 cm off.
   */
  public static double WALL_PUSH = 0.25;
  /** Sliding onto the wall, the robot counts as against it once this slow, after WALL_SETTLE_S of the slide. */
  public static double WALL_STOPPED_MPS = 0.03;
  public static double WALL_SETTLE_S = 0.3;

  private TipPredictor tips;
  /** The Cell the current shooting spot is in front of, picked as its drive started. */
  private Cell shotSpotCell;
  /** The Cell up at the start, the first volley's: the plan's later volleys alternate from it. */
  Cell firstCell;

  protected ScoringAuto(Alliance alliance) {
    super(alliance);
  }

  @Override
  protected double[] redStart() {
    return AutoField.START;
  }

  @Override
  protected AutoSequence newPlan() {
    AutoSequence sequence = new AutoSequence(0);
    tips = new TipPredictor(robot.getUpCell());
    firstCell = tips.cell();
    return sequence;
  }

  @Override
  protected String placement() {
    return alliance + ": place the robot touching our wall at the start mark, camera to the Hive";
  }

  @Override
  protected String status(double t) {
    return String.format(Locale.US, "%s  %.1f s  %d shot  aim %s%s", plan.step(), t, plan.ballsShot(), aimCell(),
        plan.isSkippingSecondFlower() ? "  (skipping 2nd Flower)" : "");
  }

  @Override
  protected void onLoopStart(double t) {
    tips.onVision(robot.getUpCell(), robot.getUpCellAgreeingFrames(), t);
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
    Logger.recordOutput("Auto/PredictedCell", tips.cell().name());
    Logger.recordOutput("Auto/SkipSecondFlower", plan.isSkippingSecondFlower());
    Logger.recordOutput("Auto/ShotSpotCell", shotSpotCell == null ? "" : shotSpotCell.name());
  }

  /**
   * The step's drive is done. Turning in place to the Pollen Pedro only holds there, so that is done once the robot
   * is on the spot and facing it: the sweep then starts facing the Pollen, and the arm doesn't swing down mid-turn.
   */
  @Override
  protected boolean stepArrived(AutoSequence.Step step, boolean facingStep, double inStepS, double robotSpeedMps) {
    if ((step == AutoSequence.Step.INTAKE_WALL_FLOWER || step == AutoSequence.Step.INTAKE_REAR_FLOWER)
        && inStepS >= FLOWER_STALL_AFTER_S && robotSpeedMps <= FLOWER_STALLED_MPS) {
      return true; // Pushed up against the Flower short of the intake point: in it (14 Nov 212840 stopped 12 cm short)
    }
    if (step == AutoSequence.Step.SWEEP_GARDEN && inStepS >= FLOWER_STALL_AFTER_S
        && robotSpeedMps <= WALL_STOPPED_MPS) {
      return true; // In the corner: the sweep drives past our wall, so the wall ends it
    }
    if (!facingStep) {
      return !follower.following();
    }
    double[] stop = AutoField.forAlliance(holdSpot(step), alliance);
    FieldPose pose = pose();
    if (step == AutoSequence.Step.ONTO_GARDEN_WALL && inStepS >= WALL_SETTLE_S
        && robotSpeedMps <= WALL_STOPPED_MPS) {
      return true; // Stopped by the wall, wherever odometry puts it
    }
    double within = step == AutoSequence.Step.ONTO_GARDEN_WALL ? ONTO_WALL_M : FACE_FLOWER_M;
    return Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= within
        && Math.abs(Angles.normalize(pose.headingRad - stop[2])) <= FACE_FLOWER_RAD;
  }

  /** The Cell to aim at: the plan's for the last shooting step started, the start's up Cell before the first. */
  @Override
  protected Cell aimCell() {
    return shotSpotCell != null ? shotSpotCell : firstCell;
  }

  @Override
  protected double flowerHeading(AutoSequence.Step step) {
    return step == AutoSequence.Step.INTAKE_WALL_FLOWER || step == AutoSequence.Step.INTAKE_REAR_FLOWER
        ? lockedHeading(step) : Double.NaN;
  }

  @Override
  protected void startStep(AutoSequence.Step step) {
    FieldPose now = pose();
    Pose here = new Pose(now.xM / Units.M_PER_IN, now.yM / Units.M_PER_IN, now.headingRad);
    boolean slow = step == AutoSequence.Step.SWEEP_GARDEN || step == AutoSequence.Step.INTAKE_WALL_FLOWER
        || step == AutoSequence.Step.INTAKE_REAR_FLOWER;
    drivetrain.setScale(slow ? SWEEP_POWER_SCALE : 1.0);
    boolean onWall = step == AutoSequence.Step.ONTO_GARDEN_WALL || step == AutoSequence.Step.SWEEP_GARDEN;
    drivetrain.setStrafeBias(onWall ? WALL_PUSH : 0.0);
    drivetrain.setForwardBias(0.0);
    lockHeading(lockedHeading(step));
    switch (step) {
      case SHOOT_PRELOAD: // To the spot in front of whichever Cell is up now, back (shooter) to it
      case SHOOT_GARDEN:
      case SHOOT_WALL_FLOWER:
        shotSpotCell = AutoSequence.plannedCell(firstCell, step);
        Pose spot = AutoField.pedroShotSpot(alliance, shotSpotCell);
        double[] control = AutoField.shotSpotControl(step, AutoField.forAlliance(AutoField.shotSpot(alliance, shotSpotCell),
            alliance));
        if (control == null || here.distance(spot) < MIN_PATH_IN) {
          line(here, spot, true);
        } else { // Round our wall's Flower
          follower.follow(Paths.curve(here, pedro(control, alliance), spot).linear(here.heading(), spot.heading()));
        }
        break;
      case DRIVE_TO_GARDEN: { // Turning on the way to face along the Pollen line
        Pose garden = pedro(AutoField.GARDEN_IN, alliance);
        double[] bend = AutoField.gardenEntryControl(AutoField.forAlliance(new double[] {now.xM, now.yM, now.headingRad},
            alliance)); // forAlliance turns blue to red as well as red to blue
        if (bend == null || here.distance(garden) < MIN_PATH_IN) {
          line(here, garden, true);
        } else { // From the far Cell's spot, below the Hive
          follower.follow(Paths.curve(here, pedro(bend, alliance), garden).linear(here.heading(), garden.heading()));
        }
        break;
      }
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
      case DRIVE_TO_REAR_FLOWER: {
        Pose end = pedro(AutoField.REAR_FLOWER, alliance);
        if (here.distance(end) < MIN_PATH_IN) {
          follower.hold(end.withHeading(here.heading()));
        } else {
          follower.follow(Paths.curve(here, pedro(AutoField.REAR_CONTROL, alliance), end)
              .linear(here.heading(), end.heading()));
        }
        break;
      }
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
      case DRIVE_TO_PARK: // Back (shooter) to the Hive, to shoot what it holds once parked
        shotSpotCell = AutoSequence.plannedCell(firstCell, AutoSequence.Step.PARKED);
        line(here, AutoField.pedroParkSpot(alliance, shotSpotCell), true);
        break;
      case PARKED: // Held off our wall, turned to the Cell, shooting until the end
        shotSpotCell = AutoSequence.plannedCell(firstCell, step);
        follower.hold(AutoField.pedroParkSpot(alliance, shotSpotCell));
        break;
    }
  }

  /**
   * The heading the heading lock holds for a step taken at a fixed heading (the Garden, a Flower), or NaN to leave
   * the turn to Pedro on the drives between places.
   */
  private double lockedHeading(AutoSequence.Step step) {
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

  /** Where a step that holds a spot (red) holds it. */
  private static double[] holdSpot(AutoSequence.Step step) {
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
}
