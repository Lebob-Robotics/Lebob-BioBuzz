package org.firstinspires.ftc.teamcode.utils;

/**
 * The robot's recent poses with their times, so a camera frame can be matched to where the robot was
 * when it was taken. A ring: the oldest is overwritten first. Times must increase. Not thread safe.
 */
public final class PoseHistory {
  private final long[] timesNanos;
  private final double[] xM;
  private final double[] yM;
  private final double[] headingsRad;
  private int oldest;
  private int count;

  public PoseHistory(int capacity) {
    int size = Math.max(1, capacity);
    timesNanos = new long[size];
    xM = new double[size];
    yM = new double[size];
    headingsRad = new double[size];
  }

  public void add(long timeNanos, Pose pose) {
    int slot;
    if (count < timesNanos.length) {
      slot = (oldest + count) % timesNanos.length;
      count++;
    } else {
      slot = oldest;
      oldest = (oldest + 1) % timesNanos.length;
    }
    timesNanos[slot] = timeNanos;
    xM[slot] = pose.xM;
    yM[slot] = pose.yM;
    headingsRad[slot] = pose.headingRad;
  }

  public int size() {
    return count;
  }

  public void clear() {
    oldest = 0;
    count = 0;
  }

  /** Moves every stored pose, for example after a vision correction. */
  public void shift(double dxM, double dyM) {
    for (int k = 0; k < count; k++) {
      int slot = slotOf(k);
      xM[slot] += dxM;
      yM[slot] += dyM;
    }
  }

  /**
   * The pose at a time, blended between the two samples around it (heading takes the short way round).
   * Up to one average sample gap outside the stored range gives the end sample; further out gives null.
   */
  public Pose sample(long timeNanos) {
    if (count == 0) {
      return null;
    }
    long first = timesNanos[slotOf(0)];
    long last = timesNanos[slotOf(count - 1)];
    long gap = count > 1 ? (last - first) / (count - 1) : 0;
    if (timeNanos < first) {
      return first - timeNanos <= gap ? at(0) : null;
    }
    if (timeNanos > last) {
      return timeNanos - last <= gap ? at(count - 1) : null;
    }
    for (int k = 0; k < count - 1; k++) {
      long before = timesNanos[slotOf(k)];
      long after = timesNanos[slotOf(k + 1)];
      if (timeNanos <= after) {
        if (after == before) {
          return at(k + 1);
        }
        double fraction = (double) (timeNanos - before) / (double) (after - before);
        int a = slotOf(k);
        int b = slotOf(k + 1);
        return new Pose(
            xM[a] + fraction * (xM[b] - xM[a]),
            yM[a] + fraction * (yM[b] - yM[a]),
            ShootOnTheMove.normalizeAngle(headingsRad[a]
                + fraction * ShootOnTheMove.normalizeAngle(headingsRad[b] - headingsRad[a])));
      }
    }
    return at(count - 1);
  }

  private int slotOf(int ageOrder) { // 0 is the oldest
    return (oldest + ageOrder) % timesNanos.length;
  }

  private Pose at(int ageOrder) {
    int slot = slotOf(ageOrder);
    return new Pose(xM[slot], yM[slot], headingsRad[slot]);
  }
}
