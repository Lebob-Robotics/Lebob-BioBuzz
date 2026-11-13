package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;

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

  /** Touching our wall, outside the loading zone and the Flower, camera side (back) toward the Hive. */
  public static final double[] START = {-0.30, -AT_WALL_M, -Math.PI / 2};
  /** The Garden sweep: front (intake) toward the audience wall, down the Pollen line toward the corner. */
  public static final double[] GARDEN_IN = {1.55, -1.05, 0};
  public static final double[] GARDEN_OUT = {1.55, -1.55, 0};
  /** Where the shooting drive to our wall's Flower ends, and where the intake then pushes in, facing the wall. */
  public static final double[] WALL_FLOWER = {0.61, -1.40, -Math.PI / 2};
  public static final double[] WALL_FLOWER_INTAKE = {0.61, -1.55, -Math.PI / 2};
  /** The rear-wall Flower likewise, facing the rear wall. */
  public static final double[] REAR_FLOWER = {-1.35, -0.61, Math.PI};
  public static final double[] REAR_FLOWER_INTAKE = {-1.55, -0.61, Math.PI};
  /** Bends the drive from our wall's Flower to the rear one round the Hive. */
  public static final double[] REAR_CONTROL = {-1.30, -1.50, 0};
  /** In the middle of the loading zone, camera side toward the Hive as at the start, so the TeleOp's placed heading holds. */
  public static final double[] PARK = {-0.915, -1.52, -Math.PI / 2};
  /** Red loading zone, tile A5: {xMin, xMax, yMin, yMax}. */
  public static final double[] LOADING_ZONE = {-1.22, -0.61, -FIELD_HALF_M, -1.22};

  private AutoField() {}

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
