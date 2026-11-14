package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner;
import org.firstinspires.ftc.teamcode.utils.VolleyCalibration;
import org.junit.Test;

public class VolleySettingsTest {
  /** CSV lines for {@code volleys} four-ball volleys at spot 0, every ball marked {@code mark}. */
  private static List<String> spotZero(int volleys, String mark) {
    VolleyAutoTuner t = new VolleyAutoTuner();
    List<String> lines = new ArrayList<>();
    lines.add(VolleyTuning.CSV_HEADER);
    for (int v = 0; v < volleys; v++) {
      VolleyAutoTuner.Settings s = t.current();
      VolleyCalibration c = t.calibration();
      c.startVolley(s.firstTrim, s.laterTrim);
      for (int i = 0; i < 4; i++) {
        c.fed();
        lines.add(VolleyTuning.resultRow("s", c, c.mark(mark), s, 0, new double[] {-1, -.32}, 2,
            "four detected balls"));
      }
      t.completed(c.balls(), 2, "four detected balls");
    }
    return lines;
  }

  @Test public void noCsvMeansTheConstants() {
    VolleySettings v = VolleySettings.fromLines(Collections.emptyList());
    assertEquals(-1, v.spotFor(new Pose(0, 0, 0), -1.5, -1.5));
    assertNull(v.get(-1));
  }

  @Test public void aTunedSpotIsUsedNearItAndGuessedEverywhereElse() {
    VolleySettings v = VolleySettings.fromLines(spotZero(3, "IN"));
    double[] p = VolleySpotDrive.point(0, -1.5, -1.5);
    assertEquals(0, v.spotFor(new Pose(p[0], p[1], 0), -1.5, -1.5));
    assertTrue(v.isNear(new Pose(p[0], p[1], 0), -1.5, -1.5, 0));
    // Far away (and the other alliance's mirror): still spot 0's settings, as the guess
    assertEquals(0, v.spotFor(new Pose(1.2, 1.2, 0), 1.5, 1.5));
    assertFalse(v.isNear(new Pose(1.2, 1.2, 0), 1.5, 1.5, 0));
    assertNotNull(v.get(0));
    assertTrue(v.isConfirmed(0)); // Good twice in a row confirms it
    assertFalse(VolleySettings.fromLines(spotZero(1, "IN")).isConfirmed(0));
  }

  @Test public void unscoredSpotGivesNoSettings() {
    VolleySettings v = VolleySettings.fromLines(spotZero(3, VolleyCalibration.UNSEEN));
    assertEquals(-1, v.spotFor(new Pose(0, 0, 0), -1.5, -1.5));
  }

  @Test public void rbIsTheVolleyAndRtAloneNeverIs() {
    assertTrue(Robot.volleyRequested(true, 0, false, false, false)); // RB
    assertTrue(Robot.volleyRequested(true, 1, false, false, false)); // RB wins over RT: holding still
    assertFalse(Robot.volleyRequested(false, 1, false, false, false)); // RT one at a time: VOLLEY_ON_RT off
    assertTrue(Robot.volleyRequested(false, 1, false, false, true)); // RT's volley on the move
    assertFalse(Robot.volleyRequested(false, 0, false, false, true));
    assertTrue(Robot.volleyRequested(false, 0, true, false, true)); // Volley Tuning's aim before RB
    assertFalse(Robot.volleyRequested(false, 1, true, false, true)); // Volley Tuning's RT stays one ball
    assertFalse(Robot.volleyRequested(true, 0, false, true, true)); // Never in Shot/SOTM Tuning
    assertFalse(Robot.volleyRequested(false, 1, false, true, true));
  }
}
