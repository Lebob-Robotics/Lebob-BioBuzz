package org.firstinspires.ftc.teamcode.field;

/**
 * The Cells, their AprilTag clusters and the ball, from the field CAD (Onshape export of 25 Oct 2026,
 * FTC2026/files) unless noted. Hive tag ids: RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING
 * 42-45. AUDIENCE Cells are at +x and SCORING at -x. Measured rather than tuned, so not on Panels.
 */
public final class CellGeometry {
  private CellGeometry() {}

  public static final double CLUSTER_TO_NEAR_LIP_M = 0.157; // Near lip is this much nearer the robot than the tag cluster centre
  public static final double MIN_UP_CELL_HEIGHT_M = 1.08; // Halfway between the up (1.26 m) and down (0.90 m) Cells' clusters
  public static final double CELL_HALF_WIDTH_M = 0.254; // Opening is 20 in wide
  public static final double BALL_RADIUS_M = 0.0356; // Pollen, 2.8 in across

  public static final double CLUSTER_UP_X_M = 0.3329; // Tag cluster centre of the up Cell: |x| and height
  public static final double CLUSTER_UP_Z_M = 1.2646;
  public static final double CLUSTER_DOWN_X_M = 0.2949; // Same for the down Cell
  public static final double CLUSTER_DOWN_Z_M = 0.9022;
  public static final double[] TAG_Y_IN = { // By tag id less 30, four per Cell
      -6.25, -10.0, -15.5, -19.25,
      -19.25, -15.5, -10.0, -6.25,
      6.25, 10.0, 15.5, 19.25,
      19.25, 15.5, 10.0, 6.25};

  // Where the SDK cluster detector puts a Cell, from the centre of its line of tags: up, and toward the
  // camera (negative is away). Measured against the SDK on the same frames; see ClusterFromTags.
  public static final double CELL_REF_UP_UP_IN = 8.0; // Cell up
  public static final double CELL_REF_UP_TOWARD_IN = 4.2;
  public static final double CELL_REF_DOWN_UP_IN = 0.5; // Cell down
  public static final double CELL_REF_DOWN_TOWARD_IN = -7.1;
}
