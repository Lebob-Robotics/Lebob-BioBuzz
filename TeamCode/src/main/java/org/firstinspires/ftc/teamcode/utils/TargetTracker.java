package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants.*;

/**
 * Remembers where our up-facing Cell is on the field, from camera sightings.
 *
 * <p>Camera frames arrive late, so each sighting is placed using the robot pose from when the frame
 * was taken. Between sightings the stored position stays put and the Pinpoint keeps the aim right as
 * the robot moves. The Cells move when the Hive tips, so nothing here is a fixed field position.
 */
public final class TargetTracker {
  private final long[] poseTimesNanos;
  private final Pose[] poses;
  private int nextPoseSlot;
  private int poseCount;

  private double cellX;
  private double cellY;
  private long lastSeenNanos;
  private boolean cellSeen;
  private String cellName = "";

  public TargetTracker(int poseHistorySize) {
    poseTimesNanos = new long[poseHistorySize];
    poses = new Pose[poseHistorySize];
  }

  /** Call once per loop. */
  public void recordPose(long timeNanos, Pose pose) {
    poseTimesNanos[nextPoseSlot] = timeNanos;
    poses[nextPoseSlot] = pose;
    nextPoseSlot = (nextPoseSlot + 1) % poses.length;
    poseCount = Math.min(poseCount + 1, poses.length);
  }

  /**
   * Places the Cell from one sighting. Returns false, and changes nothing, when there is no pose yet
   * or the cluster is too low to be the up-facing Cell.
   */
  public boolean update(CellSighting sighting) {
    if (poseCount == 0) {
      return false;
    }
    double[] inRobot = sighting.camera.toRobotFrame(sighting.rightM, sighting.forwardM, sighting.upM);
    double robotForwardM = inRobot[0];
    double robotLeftM = inRobot[1];
    if (inRobot[2] < MIN_UP_CELL_HEIGHT_M) {
      return false;
    }

    Pose pose = poseNearestTo(sighting.frameTimeNanos);
    boolean sameCell = sighting.clusterName.equals(cellName);
    if (cellSeen && !sameCell) {
      // Another of our Cells in view: keep the one we have unless the new one is clearly nearer or
      // ours hasn't been seen for a while (the Hive tipped), so the aim doesn't flip between Cells.
      boolean heldStale = (sighting.frameTimeNanos - lastSeenNanos) * 1e-9 > CELL_SWITCH_AFTER_S;
      double heldDistM = Math.hypot(cellX - pose.xM, cellY - pose.yM);
      double newDistM = Math.hypot(robotForwardM, robotLeftM);
      if (!heldStale && newDistM > heldDistM - CELL_SWITCH_MARGIN_M) {
        return false;
      }
    }
    double cosHeading = Math.cos(pose.headingRad);
    double sinHeading = Math.sin(pose.headingRad);
    double sightedX = pose.xM + robotForwardM * cosHeading - robotLeftM * sinHeading;
    double sightedY = pose.yM + robotForwardM * sinHeading + robotLeftM * cosHeading;

    double moveShare = cellSeen && sameCell ? CELL_SMOOTHING : 1.0; // A new Cell is taken as it is
    cellX += (sightedX - cellX) * moveShare;
    cellY += (sightedY - cellY) * moveShare;
    lastSeenNanos = Math.max(lastSeenNanos, sighting.frameTimeNanos);
    cellSeen = true;
    cellName = sighting.clusterName;
    return true;
  }

  private Pose poseNearestTo(long timeNanos) {
    int nearest = 0;
    for (int slot = 1; slot < poseCount; slot++) {
      if (Math.abs(poseTimesNanos[slot] - timeNanos) < Math.abs(poseTimesNanos[nearest] - timeNanos)) {
        nearest = slot;
      }
    }
    return poses[nearest];
  }

  /**
   * Moves the stored Cell and every stored pose by the same amount. Call when the pose is corrected,
   * so the Cell stays where it was relative to the robot and the aim does not jump.
   */
  public void shift(double dxM, double dyM) {
    if (cellSeen) {
      cellX += dxM;
      cellY += dyM;
    }
    for (int slot = 0; slot < poseCount; slot++) {
      Pose pose = poses[slot];
      poses[slot] = new Pose(pose.xM + dxM, pose.yM + dyM, pose.headingRad);
    }
  }

  /** True when the Cell was seen within the last CELL_EXPIRY_S. */
  public boolean knowsCell(long nowNanos) {
    return cellSeen && secondsSinceSeen(nowNanos) <= CELL_EXPIRY_S;
  }

  /** Tag cluster centre. Only meaningful when {@link #knowsCell}. */
  public double getCellX() {
    return cellX;
  }

  public double getCellY() {
    return cellY;
  }

  /** Infinity if never seen. */
  public double secondsSinceSeen(long nowNanos) {
    return cellSeen ? (nowNanos - lastSeenNanos) * 1e-9 : Double.POSITIVE_INFINITY;
  }

  /** Forgets the Cell and every pose. Call whenever the field frame moves, such as a heading reset. */
  public void clear() {
    cellSeen = false;
    cellX = 0;
    cellY = 0;
    lastSeenNanos = 0;
    poseCount = 0;
    nextPoseSlot = 0;
  }
}
