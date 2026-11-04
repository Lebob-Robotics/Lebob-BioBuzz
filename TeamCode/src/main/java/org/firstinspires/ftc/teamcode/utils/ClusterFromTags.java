package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Works out where the SDK's cluster detector would put each Cell, from the single tags alone, so only one
 * detector has to run per frame. Each Cell's four tags sit on a level line. Two or more tags give that
 * line, and the Cell reference point is a fixed offset from the line's centre: up, and toward or away
 * from the camera. The offset depends on whether the Cell is up or down. It was measured against the SDK
 * cluster detector on 4 Nov (both detectors on the same frames). Everything is in the SDK's ftcPose axes
 * (right, forward along the lens, up the picture), in inches.
 */
public final class ClusterFromTags {
  /** One Cell worked out from its tags. */
  public static final class Cluster {
    public final String name;
    public final double xIn;
    public final double yIn;
    public final double zIn;
    public final int tags;

    Cluster(String name, double xIn, double yIn, double zIn, int tags) {
      this.name = name;
      this.xIn = xIn;
      this.yIn = yIn;
      this.zIn = zIn;
      this.tags = tags;
    }
  }

  private static final String[] NAMES = {"RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"};
  // Each tag's place along its Cell's line, by id less 30 mod 4, from the SDK's cluster metadata
  private static final double[] ALONG_IN = {-6.5, -2.75, 2.75, 6.5};

  private ClusterFromTags() {}

  /** The SDK name of the Cell a Hive tag belongs to, or null. */
  public static String cellName(int id) {
    return id >= 30 && id <= 45 ? NAMES[(id - 30) / 4] : null;
  }

  /**
   * The Cells seen in one frame.
   *
   * @param ids tag ids; anything that isn't a Hive tag, and tags with no pose (NaN), are skipped
   */
  public static List<Cluster> build(int[] ids, double[] xIn, double[] yIn, double[] zIn, CameraMount camera) {
    // The robot's up direction in camera axes: the height each camera axis adds in robot terms
    double[] origin = camera.toRobotFrame(0, 0, 0);
    double[] up = {
        camera.toRobotFrame(1, 0, 0)[2] - origin[2],
        camera.toRobotFrame(0, 1, 0)[2] - origin[2],
        camera.toRobotFrame(0, 0, 1)[2] - origin[2]};

    List<Cluster> clusters = new ArrayList<>();
    for (int cell = 0; cell < NAMES.length; cell++) {
      int n = 0;
      double meanAlong = 0;
      double[] mean = new double[3];
      for (int i = 0; i < ids.length; i++) {
        if (inCell(ids[i], cell) && hasPose(xIn[i], yIn[i], zIn[i])) {
          n++;
          meanAlong += ALONG_IN[(ids[i] - 30) % 4];
          mean[0] += xIn[i];
          mean[1] += yIn[i];
          mean[2] += zIn[i];
        }
      }
      if (n < 2) {
        continue; // One tag doesn't give the line's direction
      }
      meanAlong /= n;
      for (int k = 0; k < 3; k++) {
        mean[k] /= n;
      }
      // Least-squares direction of the line, per inch along it
      double[] dir = new double[3];
      double spread = 0;
      for (int i = 0; i < ids.length; i++) {
        if (inCell(ids[i], cell) && hasPose(xIn[i], yIn[i], zIn[i])) {
          double d = ALONG_IN[(ids[i] - 30) % 4] - meanAlong;
          spread += d * d;
          dir[0] += d * (xIn[i] - mean[0]);
          dir[1] += d * (yIn[i] - mean[1]);
          dir[2] += d * (zIn[i] - mean[2]);
        }
      }
      if (spread <= 0) {
        continue;
      }
      double[] centre = new double[3];
      for (int k = 0; k < 3; k++) {
        dir[k] /= spread;
        centre[k] = mean[k] - meanAlong * dir[k];
      }
      // The line is level, so drop any tilt the noise gave it, then build up and toward-camera axes
      double[] along = normalize(subtract(dir, scale(up, dot(dir, up))));
      double[] towardCamera = cross(along, up);
      if (dot(towardCamera, centre) > 0) {
        towardCamera = scale(towardCamera, -1);
      }
      double heightM = camera.toRobotFrame(centre[0] * IN_TO_M, centre[1] * IN_TO_M, centre[2] * IN_TO_M)[2];
      boolean cellUp = Math.abs(heightM - CLUSTER_UP_Z_M) < Math.abs(heightM - CLUSTER_DOWN_Z_M);
      double upIn = cellUp ? CELL_REF_UP_UP_IN : CELL_REF_DOWN_UP_IN;
      double towardIn = cellUp ? CELL_REF_UP_TOWARD_IN : CELL_REF_DOWN_TOWARD_IN;
      clusters.add(new Cluster(NAMES[cell],
          centre[0] + upIn * up[0] + towardIn * towardCamera[0],
          centre[1] + upIn * up[1] + towardIn * towardCamera[1],
          centre[2] + upIn * up[2] + towardIn * towardCamera[2], n));
    }
    return clusters;
  }

  private static boolean inCell(int id, int cell) {
    return id >= 30 && id <= 45 && (id - 30) / 4 == cell;
  }

  private static boolean hasPose(double x, double y, double z) {
    return !Double.isNaN(x) && !Double.isNaN(y) && !Double.isNaN(z);
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private static double[] scale(double[] a, double s) {
    return new double[] {a[0] * s, a[1] * s, a[2] * s};
  }

  private static double[] subtract(double[] a, double[] b) {
    return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  private static double[] cross(double[] a, double[] b) {
    return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  }

  private static double[] normalize(double[] a) {
    return scale(a, 1 / Math.sqrt(dot(a, a)));
  }
}
