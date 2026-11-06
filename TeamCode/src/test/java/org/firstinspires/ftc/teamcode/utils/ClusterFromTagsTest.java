package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class ClusterFromTagsTest {
  // One logged frame (4 Nov, right camera) where the SDK cluster detector and the single-tag detector
  // both ran. Singles first, then the SDK's cluster positions.
  private static final int[] IDS = {30, 31, 34, 36, 37, 44, 45};
  private static final double[] X = {-10.066, -13.529, -32.644, -23.941, -20.539, 5.314, 1.794};
  private static final double[] Y = {71.270, 69.644, 75.435, 78.163, 79.706, 75.092, 75.217};
  private static final double[] Z = {3.174, 2.121, 25.697, 28.389, 29.663, 7.796, 6.896};

  private static ClusterFromTags.Cluster find(List<ClusterFromTags.Cluster> clusters, String name) {
    for (ClusterFromTags.Cluster c : clusters) {
      if (c.name.equals(name)) {
        return c;
      }
    }
    throw new AssertionError("no " + name);
  }

  private static void assertNear(double[] sdk, ClusterFromTags.Cluster c, double toleranceIn) {
    double error = Math.sqrt(Math.pow(c.xIn - sdk[0], 2) + Math.pow(c.yIn - sdk[1], 2) + Math.pow(c.zIn - sdk[2], 2));
    assertTrue(c.name + " is " + error + " in from the SDK's", error < toleranceIn);
  }

  @Test
  public void matchesTheSdkClusterDetector() {
    List<ClusterFromTags.Cluster> clusters =
        ClusterFromTags.build(IDS, X, Y, Z, ShootOnTheMoveConstants.RIGHT_CAMERA);
    assertEquals(3, clusters.size());
    assertNear(new double[] {-14.201, 70.876, -7.186}, find(clusters, "RED SCORING"), 1.5); // Up, two close tags
    assertNear(new double[] {-29.346, 82.612, 31.533}, find(clusters, "RED AUDIENCE"), 1.5); // Down
    assertNear(new double[] {9.437, 78.253, -0.076}, find(clusters, "BLUE SCORING"), 1.5);
  }

  @Test
  public void needsTwoTagsOfACell() {
    List<ClusterFromTags.Cluster> clusters = ClusterFromTags.build(new int[] {30, 34, 20},
        new double[] {1, 2, 3}, new double[] {50, 50, 50}, new double[] {0, 0, 0},
        ShootOnTheMoveConstants.RIGHT_CAMERA);
    assertTrue(clusters.isEmpty());
  }
}
