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

/**
 * The corner autonomous ({@link CornerSequence}), for one alliance, driven as {@link BaseAuto} drives: preloads from
 * the rear corner, a wait there, then our wall's Flower, a volley into the AUDIENCE Cell from in front of it, and
 * park, shooting what's left.
 */
public abstract class CornerAuto extends BaseAuto<CornerSequence.Step, CornerSequence> {
  protected CornerAuto(Alliance alliance) {
    super(alliance);
  }

  @Override
  protected double[] redStart() {
    return CornerSequence.CORNER;
  }

  @Override
  protected CornerSequence newPlan() {
    return new CornerSequence(0);
  }

  /** CornerSequence's jiggle. */
  @Override
  protected boolean indexerPauseRunsBackward() {
    return true;
  }

  @Override
  protected String placement() {
    return alliance + " CORNER: robot in the rear corner, touching our wall and the rear wall,"
        + " intake to our wall, outside the loading zone";
  }

  @Override
  protected String status(double t) {
    return String.format(Locale.US, "CORNER %s  %.1f s  %d shot  aim %s", plan.step(), t, plan.ballsShot(),
        aimCell());
  }

  /** The step's drive is done: for a facing step, on its spot and at its heading. */
  @Override
  protected boolean stepArrived(CornerSequence.Step step, boolean facingStep, double inStepS, double robotSpeedMps) {
    if (step == CornerSequence.Step.INTAKE_FLOWER && inStepS >= FLOWER_STALL_AFTER_S
        && robotSpeedMps <= FLOWER_STALLED_MPS) {
      return true; // Pushed up against the Flower short of the intake point: in it
    }
    if (!facingStep) {
      return !follower.following();
    }
    double[] stop = step == CornerSequence.Step.FACE_WALL ? CornerSequence.preloadSpot(alliance)
        : AutoField.forAlliance(AutoField.WALL_FLOWER, alliance);
    FieldPose pose = pose();
    return Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= FACE_FLOWER_M
        && Math.abs(Angles.normalize(pose.headingRad - wallHeading())) <= FACE_FLOWER_RAD;
  }

  @Override
  protected Cell aimCell() {
    return aimCell(plan == null ? null : plan.step());
  }

  /** The Cell to aim at in a step; null, before the start, is the preloads'. */
  Cell aimCell(CornerSequence.Step current) {
    if (current == null) {
      return CornerSequence.nearCell(alliance);
    }
    int step = current.ordinal();
    return step >= CornerSequence.Step.DRIVE_TO_FLOWER.ordinal()
        ? CornerSequence.gardenCell(alliance) : CornerSequence.nearCell(alliance);
  }

  @Override
  protected double flowerHeading(CornerSequence.Step step) {
    return step == CornerSequence.Step.INTAKE_FLOWER ? wallHeading() : Double.NaN;
  }

  /** Intake to our wall: the start's heading, the Flower's, and the one it waits at. */
  private double wallHeading() {
    return AutoField.forAlliance(CornerSequence.CORNER, alliance)[2];
  }

  @Override
  protected void startStep(CornerSequence.Step step) {
    FieldPose now = pose();
    Pose here = new Pose(now.xM / Units.M_PER_IN, now.yM / Units.M_PER_IN, now.headingRad);
    boolean intoFlower = step == CornerSequence.Step.INTAKE_FLOWER;
    drivetrain.setScale(intoFlower ? SWEEP_POWER_SCALE : 1.0);
    drivetrain.setStrafeBias(0.0);
    drivetrain.setForwardBias(0.0);
    // Square to our wall everywhere but the AUDIENCE volley and the drive back from it, which Pedro turns on the
    // way. Shooting, the aim's lock turns the robot instead
    boolean turning = step == CornerSequence.Step.SHOOT_GARDEN_SIDE || step == CornerSequence.Step.DRIVE_TO_PARK;
    lockHeading(turning ? Double.NaN : wallHeading());
    Pose preload = poseOf(CornerSequence.preloadSpot(alliance));
    Pose flowerStop = pedro(AutoField.WALL_FLOWER, alliance);
    switch (step) {
      case SHOOT_PRELOAD: // Straight out of the corner without turning: turned in it, a corner would hit a wall
        line(here, preload, false);
        break;
      case FACE_WALL:
        follower.hold(preload);
        break;
      case BACK_TO_CORNER:
        line(here, pedro(CornerSequence.CORNER_WAIT, alliance), false);
        break;
      case WAIT_IN_CORNER:
        follower.hold(pedro(CornerSequence.CORNER_WAIT, alliance));
        break;
      case DRIVE_TO_FLOWER: { // Along our wall just off it, then up round the Flower to the stop short of it
        Pose edgeEnd = pedro(CornerSequence.EDGE_END, alliance);
        if (here.distance(edgeEnd) < MIN_PATH_IN) {
          follower.follow(Paths.curve(here, pedro(CornerSequence.FLOWER_BEND, alliance), flowerStop)
              .constant(flowerStop.heading()));
        } else {
          follower.follow(Paths.path(Paths.line(here, edgeEnd).constant(edgeEnd.heading()),
              Paths.curve(edgeEnd, pedro(CornerSequence.FLOWER_BEND, alliance), flowerStop)
                  .constant(flowerStop.heading())));
        }
        break;
      }
      case LOWER_ARM:
        follower.hold(flowerStop);
        break;
      case INTAKE_FLOWER: // Arm down, straight in still facing it
        line(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_FLOWER:
        line(here, flowerStop, false);
        break;
      case RAISE_ARM:
        follower.hold(here);
        break;
      case SHOOT_GARDEN_SIDE: // Back (shooter) to the AUDIENCE Cell, from the spot in front of it
        line(here, AutoField.pedroShotSpot(alliance, CornerSequence.gardenCell(alliance)), true);
        break;
      case DRIVE_TO_PARK: // Turning square to our wall on the way
        line(here, poseOf(CornerSequence.parkSpot(alliance)), true);
        break;
      case PARKED:
        follower.hold(poseOf(CornerSequence.parkSpot(alliance)));
        break;
    }
  }

  private static Pose poseOf(double[] field) {
    return new Pose(field[0] / Units.M_PER_IN, field[1] / Units.M_PER_IN, field[2]);
  }
}
