package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.junit.Test;

public class SpecialShotCalibrationTest {
  @Test public void spotIsOnTheHiveLineHalfAMetreOutFromTheNearLip() {
    for (Alliance alliance : Alliance.values()) {
      for (VisionCorrector.Cell far : VisionCorrector.Cell.values()) {
        double[] target = VisionCorrector.upCellAimPoint(alliance, far);
        double[] spot = SpecialShotCalibration.spotFor(alliance, far);
        // The near Cell mirrors the far one; the robot stands past its lip, on the far Cell's side of zero opposite
        double nearLipX = -target[0] - Math.signum(target[0]) * ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
        assertEquals(nearLipX - Math.signum(target[0]) * SpecialShotCalibration.LIP_GAP_M, spot[0], 1e-9);
        assertEquals(target[1], spot[1], 1e-9);
        assertEquals(true, FieldRoute.inBox(spot[0], spot[1], 0, 0));
      }
    }
  }
}
