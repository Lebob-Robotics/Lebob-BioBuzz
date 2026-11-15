package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.field.CellGeometry;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldRoute;

public class SpecialShotCalibrationTest {
  @Test public void spotIsOnTheHiveLineHalfAMetreOutFromTheNearLip() {
    for (Alliance alliance : Alliance.values()) {
      for (Cell far : Cell.values()) {
        double[] target = VisionCorrector.upCellAimPoint(alliance, far);
        double[] spot = SpecialShotCalibration.spotFor(alliance, far);
        // The near Cell mirrors the far one; the robot stands past its lip, on the far Cell's side of zero opposite
        double nearLipX = -target[0] - Math.signum(target[0]) * CellGeometry.CLUSTER_TO_NEAR_LIP_M;
        assertEquals(nearLipX - Math.signum(target[0]) * SpecialShotCalibration.LIP_GAP_M, spot[0], 1e-9);
        assertEquals(target[1], spot[1], 1e-9);
        assertEquals(true, FieldRoute.inBox(spot[0], spot[1], 0, 0));
      }
    }
  }
}
