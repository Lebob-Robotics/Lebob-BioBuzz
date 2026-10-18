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
    CameraMount camera = sighting.camera;

    double cosPitch = Math.cos(camera.pitchRad); // Undo the camera's tilt
    double sinPitch = Math.sin(camera.pitchRad);
    double levelForwardM = sighting.forwardM * cosPitch - sighting.upM * sinPitch;
    double levelUpM = sighting.forwardM * sinPitch + sighting.upM * cosPitch;
    double levelLeftM = -sighting.rightM;
    if (camera.heightM + levelUpM < MIN_UP_CELL_HEIGHT_M) {
      return false;
    }

    double cosYaw = Math.cos(camera.yawRad); // Turn by the camera's yaw, then add where it is mounted
    double sinYaw = Math.sin(camera.yawRad);
    double robotForwardM = camera.forwardM + levelForwardM * cosYaw - levelLeftM * sinYaw;
    double robotLeftM = camera.leftM + levelForwardM * sinYaw + levelLeftM * cosYaw;

    Pose pose = poseNearestTo(sighting.frameTimeNanos);
    double cosHeading = Math.cos(pose.headingRad);
    double sinHeading = Math.sin(pose.headingRad);
    double sightedX = pose.xM + robotForwardM * cosHeading - robotLeftM * sinHeading;
    double sightedY = pose.yM + robotForwardM * sinHeading + robotLeftM * cosHeading;

    double moveShare = cellSeen ? CELL_SMOOTHING : 1.0; // The first sighting is taken as it is
    cellX += (sightedX - cellX) * moveShare;
    cellY += (sightedY - cellY) * moveShare;
    lastSeenNanos = Math.max(lastSeenNanos, sighting.frameTimeNanos);
    cellSeen = true;
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
