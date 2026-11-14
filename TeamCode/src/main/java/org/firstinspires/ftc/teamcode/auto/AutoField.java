package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;

/**
 * Where the autonomous goes, as {x, y, heading} for the robot's centre: metres and radians, red alliance. Blue is
 * the same turned half a turn about the middle, as the field is (x toward the audience, red wall at -y).
 *
 * <p>From the game manual's field: red Garden tile A1 (the audience corner of our wall, Pollen along the audience
 * wall), red loading zone A5, our wall's Flower at x 0.61. During AUTO the Flower on the audience wall is on blue's
 * side (G402), so red's second Flower is the rear-wall one, near the loading zone. All are estimates until measured
 * on a real field.
 */
public final class AutoField {
  public static final double FIELD_HALF_M = 1.83;
  /** Robot centre to its bumper. Measure. */
  public static final double ROBOT_HALF_M = 0.23;
  /** Centre against a wall, bumper touching. */
  public static final double AT_WALL_M = FIELD_HALF_M - ROBOT_HALF_M;

  /**
   * Where the robot is put at the start: intake against our wall right next to the Garden, its side 2 cm from the
   * Pollen line along the audience wall (balls 3 in. against it), camera side (back) toward the Hive. G304:
   * touching the perimeter wall, not in the LOADING ZONE, not touching a FLOWER. Until 14 Nov 213317 it was put in
   * the loading zone (illegal), and the camera's late correction made the preloads shoot 0.6 m short of the spot.
   */
  public static final double[] START = {FIELD_HALF_M - 0.076 - 0.02 - ROBOT_HALF_M, -AT_WALL_M, -Math.PI / 2};
  /** Heading of the Garden sweep: front (intake) toward our wall, driving along the audience wall to the corner. */
  public static final double GARDEN_HEADING = -Math.PI / 2;
  /**
   * The drive to the Garden ends here, the robot turning on the way to {@link #GARDEN_HEADING}: clear of the audience
   * wall, as a turning robot's corners reach out its half-diagonal (8.5 cm to spare).
   */
  public static final double[] GARDEN_IN = {1.42, -0.95, GARDEN_HEADING};
  /** The sweep slides sideways onto the audience wall here, then drives along it, against it, to the end. */
  public static final double[] GARDEN_WALL = {AT_WALL_M, -0.95, GARDEN_HEADING};
  /** Along the wall into the corner, 2 cm short of our wall. */
  public static final double[] GARDEN_OUT = {AT_WALL_M, -AT_WALL_M + 0.02, GARDEN_HEADING};
  /**
   * The sweep drives for this far past {@link #GARDEN_OUT}, into our wall, and ends once the wall stops it: aimed
   * 2 cm short, odometry's drift left it short of the corner's last Pollen (14 Nov 214945).
   */
  public static double GARDEN_PUSH_M = 0.10;

  /** Where the sweep drives to: into the corner, past our wall. */
  public static double[] gardenSweepEnd() {
    return new double[] {GARDEN_OUT[0], GARDEN_OUT[1] - GARDEN_PUSH_M, GARDEN_OUT[2]};
  }
  /**
   * How far short of a Flower's intake point the drive to it stops, for the Pollen arm to come down clear of the
   * Flower. 0.25 m was too close (14 Nov run).
   */
  public static final double FLOWER_STANDOFF_M = 0.45;
  /**
   * Where the intake pushes into our wall's Flower with the arm down, facing the wall, and where the drive
   * to it stops first, {@link #FLOWER_STANDOFF_M} out. Backing out returns to the stop.
   */
  public static final double[] WALL_FLOWER_INTAKE = {0.61, -1.55, -Math.PI / 2};
  public static final double[] WALL_FLOWER = {0.61, WALL_FLOWER_INTAKE[1] + FLOWER_STANDOFF_M, -Math.PI / 2};
  /** The rear-wall Flower likewise, facing the rear wall. */
  public static final double[] REAR_FLOWER_INTAKE = {-1.55, -0.61, Math.PI};
  public static final double[] REAR_FLOWER = {REAR_FLOWER_INTAKE[0] + FLOWER_STANDOFF_M, -0.61, Math.PI};
  /**
   * Bends the drive from the Garden to a shooting spot up between the Hive and our wall's Flower: straight, 14 Nov it
   * passed the Flower 1.45 m out from the middle and hit it. Curved, it passes 1.21 m out or more.
   */
  public static final double[] GARDEN_EXIT_CONTROL = {1.40, -0.95, 0};
  /**
   * Robot centre keeps out of this box ({xMin, xMax, yMax}, toward our wall from yMax) except taking the Flower:
   * the Flower at x 0.61 reached by the robot's half-width either side, a little clearance. Estimate: measure.
   */
  public static final double[] WALL_FLOWER_KEEP_OUT = {0.23, 0.99, -1.36};

  /**
   * Bends the drive to the Garden from the far Cell's shooting spot (red x < 0) below the Hive: from
   * {@link #VOLLEY_SPOT}, 5 cm outside the Hive's route clearance, a straight line ran into it.
   */
  public static final double[] GARDEN_ENTRY_CONTROL = {0.20, -1.30, 0};

  /** The bend for the drive to the Garden from here (red coordinates), or null for a straight one. */
  public static double[] gardenEntryControl(double[] redFrom) {
    return redFrom[0] < 0 ? GARDEN_ENTRY_CONTROL : null;
  }

  /** Bends the drive from a shooting spot to the rear Flower round the Hive. */
  public static final double[] REAR_CONTROL = {-1.30, -1.50, 0};
  /** In the middle of the loading zone, camera side toward the Hive as at the start, so the TeleOp's placed heading holds. */
  public static final double[] PARK = {-0.915, -1.52, -Math.PI / 2};
  /** Pushed against our wall once parked, as power toward it. */
  public static double PARK_PUSH = 0.15;

  /**
   * Where to park, for the alliance (field coordinates, as {@link #shotSpot}): {@link #PARK}'s x, back (shooter) to
   * this Cell so the leftovers can be shot, and as close to our wall as the turned robot's corner allows: it then
   * touches the wall, pushed by {@link #PARK_PUSH}.
   */
  public static double[] parkSpot(Alliance alliance, Cell cell) {
    double[] base = forAlliance(PARK, alliance);
    double[] aim = autoAimPoint(alliance, cell);
    double ourWall = alliance == Alliance.RED ? -1 : 1;
    double y = base[1];
    double heading = base[2];
    for (int i = 0; i < 3; i++) { // The turn sets the corner's reach, which moves the spot, which moves the turn
      heading = Math.atan2(y - aim[1], base[0] - aim[0]);
      double reach = ROBOT_HALF_M * (Math.abs(Math.sin(heading)) + Math.abs(Math.cos(heading)));
      y = ourWall * (FIELD_HALF_M - reach);
    }
    return new double[] {base[0], y, heading};
  }

  /** As a Pedro pose (inches). */
  public static Pose pedroParkSpot(Alliance alliance, Cell cell) {
    double[] p = parkSpot(alliance, cell);
    return new Pose(p[0] / M_PER_IN, p[1] / M_PER_IN, p[2]);
  }

  /** {forward, strafe left} drive powers, for a robot at this heading, that push it toward our wall at {@code power}. */
  public static double[] wallPush(Alliance alliance, double headingRad, double power) {
    double ourWall = alliance == Alliance.RED ? -1 : 1;
    return new double[] {power * ourWall * Math.sin(headingRad), power * ourWall * Math.cos(headingRad)};
  }

  /** Red loading zone, tile A5: {xMin, xMax, yMin, yMax}. */
  public static final double[] LOADING_ZONE = {-1.22, -0.61, -FIELD_HALF_M, -1.22};

  private AutoField() {}

  /**
   * Where the auto shoots from, for red's SCORING Cell ({x, y}, metres): Volley Tuning's spot 4 (FieldRoute.GRID[4],
   * "Spot 5/10" on the Driver Station), 0.92 m from the aim point and 37 degrees off straight out, away from the
   * middle. Its volleys there went 46 of 52 in, 0.74 m from the lip, no double balls (14 Nov 224053). The old spot,
   * 1.0 m and 20 degrees off, now and then lost a ball outward.
   */
  public static final double[] VOLLEY_SPOT = {-1.00, -1.05};

  /**
   * Where to stop and shoot at this Cell of our Hive, in field coordinates for the alliance (not rotated from red:
   * both alliances' AUDIENCE Cells are at +x): {@link #VOLLEY_SPOT} mirrored to the Cell's side as Volley Tuning
   * mirrors it, back (shooter) to the Cell.
   */
  public static double[] shotSpot(Alliance alliance, Cell cell) {
    double[] aim = VisionCorrector.upCellAimPoint(alliance, cell);
    double x = VOLLEY_SPOT[0] * (aim[0] < 0 ? 1 : -1);
    double y = VOLLEY_SPOT[1] * (aim[1] < 0 ? 1 : -1);
    return new double[] {x, y, Math.atan2(y - aim[1], x - aim[0])}; // Front away from the Cell, back to it
  }

  /**
   * The auto aims this far from the Cell's aim point toward the field's middle. 0: Volley Tuning aims at the aim
   * point itself (with the same HOODED_HEADING_TRIM_DEG). 0.03 made up for outward misses from the old spot.
   */
  public static double AIM_INWARD_M = 0.0;

  /** Where the auto aims at this Cell: its aim point, {@link #AIM_INWARD_M} toward the middle. */
  public static double[] autoAimPoint(Alliance alliance, Cell cell) {
    double[] aim = VisionCorrector.upCellAimPoint(alliance, cell);
    return new double[] {aim[0] - Math.signum(aim[0]) * AIM_INWARD_M, aim[1]};
  }

  /** As a Pedro pose (inches). */
  public static Pose pedroShotSpot(Alliance alliance, Cell cell) {
    double[] p = shotSpot(alliance, cell);
    return new Pose(p[0] / M_PER_IN, p[1] / M_PER_IN, p[2]);
  }

  /**
   * The bend for the drive to a shooting spot clear of our wall's Flower (red coordinates), or null for a straight
   * one: from the Garden and from the start beside it. The spot is in red coordinates.
   */
  public static double[] shotSpotControl(AutoSequence.Step step, double[] redSpot) {
    if (step == AutoSequence.Step.SHOOT_GARDEN || step == AutoSequence.Step.SHOOT_PRELOAD) {
      return AutoField.GARDEN_EXIT_CONTROL; // The start is in the Garden's corner too
    }
    return null;
  }

  /** A red {x, y, heading} for this alliance. */
  public static double[] forAlliance(double[] red, Alliance alliance) {
    if (alliance == Alliance.RED) {
      return red.clone();
    }
    return new double[] {-red[0], -red[1], normalize(red[2] + Math.PI)};
  }

  /** A red {x, y, heading} for this alliance, as a Pedro pose (inches). */
  public static Pose pedro(double[] red, Alliance alliance) {
    double[] p = forAlliance(red, alliance);
    return new Pose(p[0] / M_PER_IN, p[1] / M_PER_IN, p[2]);
  }

  public static final double M_PER_IN = 0.0254;

  static double normalize(double rad) {
    return Math.atan2(Math.sin(rad), Math.cos(rad));
  }
}
