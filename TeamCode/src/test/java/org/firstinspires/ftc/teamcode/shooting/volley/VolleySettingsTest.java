package org.firstinspires.ftc.teamcode.shooting.volley;

import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.opmodes.tuning.VolleyTuning;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class VolleySettingsTest {
  /** CSV lines for {@code volleys} four-ball volleys at spot 0, every ball marked {@code mark}. */
  private static List<String> spotZero(int volleys, String mark) {
    return spotRows(0, volleys, mark);
  }

  private static List<String> spotRows(int spot, int volleys, String mark) {
    VolleyAutoTuner t = new VolleyAutoTuner();
    List<String> lines = new ArrayList<>();
    lines.add(VolleyTuning.CSV_HEADER);
    for (int v = 0; v < volleys; v++) {
      VolleyAutoTuner.Settings s = t.current();
      VolleyCalibration c = t.calibration();
      c.startVolley(s.firstTrim, s.laterTrim);
      for (int i = 0; i < 4; i++) {
        c.fed();
        lines.add(VolleyTuning.resultRow("s", c, c.mark(mark), s, spot, new double[] {-1, -.32}, 2,
            "four detected balls"));
      }
      t.completed(c.balls(), 2, "four detected balls");
    }
    return lines;
  }

  @Test public void noCsvMeansTheConstants() {
    VolleySettings v = VolleySettings.fromLines(Collections.emptyList());
    assertEquals(-1, v.spotFor(new FieldPose(0, 0, 0), -1.5, -1.5));
    assertNull(v.get(-1));
  }

  @Test public void aTunedSpotIsUsedNearItAndGuessedEverywhereElse() {
    VolleySettings v = VolleySettings.fromLines(spotZero(3, "IN"));
    double[] p = VolleySpotDrive.point(0, -1.5, -1.5);
    assertEquals(0, v.spotFor(new FieldPose(p[0], p[1], 0), -1.5, -1.5));
    assertTrue(v.isNear(new FieldPose(p[0], p[1], 0), -1.5, -1.5, 0));
    // Far away (and the other alliance's mirror): still spot 0's settings, as the guess
    assertEquals(0, v.spotFor(new FieldPose(1.2, 1.2, 0), 1.5, 1.5));
    assertFalse(v.isNear(new FieldPose(1.2, 1.2, 0), 1.5, 1.5, 0));
    assertNotNull(v.get(0));
    assertTrue(v.isConfirmed(0)); // Good twice in a row confirms it
    assertFalse(VolleySettings.fromLines(spotZero(1, "IN")).isConfirmed(0));
  }

  @Test public void anUnconfirmedSpotIsNotUsed() {
    // One good volley is a best-so-far, which can be from an old feed (spot 0's 12 Nov 0.6 feeder): constants instead
    assertEquals(-1, VolleySettings.fromLines(spotZero(1, "IN")).spotFor(new FieldPose(-1, -0.32, 0), -1.5, -1.5));
  }

  @Test public void awayFromEveryTunedSpotTheNearestConfirmedOneIsUsed() throws Exception {
    List<String> lines = new ArrayList<>();
    try (java.io.BufferedReader in = new java.io.BufferedReader(new java.io.InputStreamReader(
        getClass().getResourceAsStream("/volley/VolleyTuning_20261115.csv"), "UTF-8"))) {
      for (String l = in.readLine(); l != null; l = in.readLine()) { lines.add(l); }
    }
    VolleySettings v = VolleySettings.fromLines(lines);
    assertNull(v.get(0)); // Spot 0 is unconfirmed old data
    // 0.6 m past spot 9, the farthest out: spot 9's, though over NEAR_SPOT_M from it
    double[] p9 = VolleySpotDrive.point(9, -1.5, -1.5);
    FieldPose far = new FieldPose(p9[0], p9[1] + 0.6, 0);
    assertEquals(9, v.spotFor(far, -1.5, -1.5));
    assertFalse(v.isNear(far, -1.5, -1.5, 9));
    assertTrue(v.isConfirmed(6)); // Spot 6 confirmed at feed profile 1 this morning
    assertEquals(0.75, v.get(6).indexerScale, 1e-9);
    assertTrue(v.isConfirmed(VolleySpotDrive.PARK)); // The park is loaded too
  }

  @Test public void awayFromEverySpotTheDistanceToTheCellPicks() {
    boolean[] known = new boolean[VolleySpotDrive.SPOTS];
    known[4] = true; known[9] = true; // 0.76 m and 1.57 m from the lip
    double[] cell = {-0.44, -0.32};
    // 1.8 m from the Cell on the far side of it: spot 9's distance, though nowhere near spot 9
    assertEquals(9, VolleySettings.nearestRange(new FieldPose(1.2, 0.5, 0), cell[0], cell[1], known));
    assertEquals(4, VolleySettings.nearestRange(new FieldPose(0.5, -0.32, 0), cell[0], cell[1], known));
  }

  @Test public void backwardSpotsOnlyServeBackwardVolleys() {
    int b3 = VolleySpotDrive.BACKWARD_FIRST + 2;
    VolleySettings v = VolleySettings.fromLines(spotRows(b3, 3, "IN"));
    assertTrue(v.isConfirmed(b3));
    double[] p = VolleySpotDrive.point(b3, -1.5, -1.5);
    FieldPose there = new FieldPose(p[0], p[1], 0);
    assertEquals(b3, v.spotFor(there, -1.5, -1.5, true));
    assertEquals(-1, v.spotFor(there, -1.5, -1.5)); // A hooded volley never takes a backward spot's settings
    VolleySettings hooded = VolleySettings.fromLines(spotZero(3, "IN"));
    assertEquals(-1, hooded.spotFor(there, -1.5, -1.5, true));
  }

  @Test public void unscoredSpotGivesNoSettings() {
    VolleySettings v = VolleySettings.fromLines(spotZero(3, VolleyCalibration.UNSEEN));
    assertEquals(-1, v.spotFor(new FieldPose(0, 0, 0), -1.5, -1.5));
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
