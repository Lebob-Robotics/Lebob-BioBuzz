package org.firstinspires.ftc.teamcode.shooting.volley;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.auto.AutoField;
import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * Keeps the auto's and TeleOp's volleys as Volley Tuning scored them on 14 Nov: profile 0 (full feeder, full
 * indexer, no hood trims, no boost) went 8 of 8 at spot 4 (224053) and 4 of 4 at the park against our wall
 * (230159, 1.11 m from the lip, 3500 RPM, hood 57.9). Change these only with new Volley Tuning results.
 */
public class TunedVolleyLockTest {
  @Test public void volleysFeedAsTheTunedProfile() {
    assertEquals(1.0, VolleyConstants.VOLLEY_FEED_POWER, 0);
    assertEquals(1.0, VolleyConstants.VOLLEY_INDEXER_SCALE, 0);
    assertEquals(0, VolleyConstants.VOLLEY_BOOST_RPM, 0);
    // Without the CSV: the medians of the eight forward spots confirmed by 15 Nov 1140
    assertEquals(-0.75, VolleyConstants.VOLLEY_FIRST_BALL_TRIM_DEG, 0);
    assertEquals(-0.75, VolleyConstants.VOLLEY_LATER_BALL_TRIM_DEG, 0);
    assertEquals(0, AimConstants.HOODED_RPM_OFFSET, 0);
    assertEquals("profile 0 fed 0.30 s apart or more", 0.30, VolleyConstants.VOLLEY_MIN_FEED_SPACING_S, 0);
  }

  @Test public void theAutoAimsAsVolleyTuningDid() {
    assertEquals(2.0, AimConstants.HOODED_HEADING_TRIM_DEG, 0); // Volley Tuning adds 0 to it
    assertEquals(0, AutoField.AIM_INWARD_M, 0);
  }

  @Test public void theAutoShootsFromTheTunedSpots() {
    for (Cell cell : Cell.values()) {
      double[] aim = VisionCorrector.upCellAimPoint(Alliance.RED, cell);
      double[] spot = AutoField.shotSpot(Alliance.RED, cell);
      assertArrayEquals("spot 4", VolleySpotDrive.point(4, aim[0], aim[1]), new double[] {spot[0], spot[1]}, 1e-9);
      if (aim[0] < 0) {
        double[] park = AutoField.parkSpot(Alliance.RED, cell);
        assertArrayEquals("park", VolleySpotDrive.point(VolleySpotDrive.PARK, aim[0], aim[1]),
            new double[] {park[0], park[1]}, 1e-9);
      }
    }
  }
}
