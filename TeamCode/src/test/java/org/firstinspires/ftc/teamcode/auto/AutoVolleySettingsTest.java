package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySettings;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySettingsAccess;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySpotDrive;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyAutoTuner.Settings;
import org.junit.Test;

/** The scoring auto shoots with Volley Tuning's spots: the 15 Nov competition file. */
public class AutoVolleySettingsTest {
  static VolleySettings comp() throws Exception {
    List<String> lines = new ArrayList<>();
    try (BufferedReader in = new BufferedReader(new InputStreamReader(
        AutoVolleySettingsTest.class.getResourceAsStream("/volley/VolleyTuning_20261115.csv"), "UTF-8"))) {
      for (String l = in.readLine(); l != null; l = in.readLine()) { lines.add(l); }
    }
    return VolleySettingsAccess.fromLines(lines);
  }

  @Test public void everyShotSpotUsesTunedSpotFour() throws Exception {
    VolleySettings v = comp();
    for (Alliance a : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
      for (Cell c : Cell.values()) {
        double[] s = AutoField.shotSpot(a, c);
        double[] aim = AutoField.autoAimPoint(a, c);
        int spot = v.spotFor(new FieldPose(s[0], s[1], s[2]), aim[0], aim[1]);
        assertEquals(a + " " + c, 4, spot);
        assertEquals(v.isNear(new FieldPose(s[0], s[1], s[2]), aim[0], aim[1], spot), true);
        Settings t = v.get(spot);
        assertEquals(-0.5, t.firstTrim, 1e-9);
        assertEquals(-0.5, t.laterTrim, 1e-9);
      }
    }
  }

  @Test public void everySoloVolleyIsShotFromAConfirmedTunedSpot() throws Exception {
    VolleySettings v = comp();
    for (Alliance a : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
      Cell near = SoloSequence.nearCell(a);
      Cell far = SoloSequence.parkCell(a);
      Object[][] volleys = {
          {"preloads", SoloSequence.preloadSpot(a), near, 4},
          {"near Cell", AutoField.shotSpot(a, near), near, 4},
          {"far Cell", AutoField.shotSpot(a, far), far, 4},
      };
      for (Object[] volley : volleys) {
        double[] s = (double[]) volley[1];
        double[] aim = AutoField.autoAimPoint(a, (Cell) volley[2]);
        int want = (int) volley[3];
        String what = a + " " + volley[0];
        assertEquals(what, want, v.spotFor(new FieldPose(s[0], s[1], s[2]), aim[0], aim[1]));
        assertTrue(what + " confirmed", VolleySettingsAccess.isConfirmed(v, want));
        double[] tuned = VolleySpotDrive.point(want, aim[0], aim[1]);
        assertEquals(what + " on the spot", 0, Math.hypot(s[0] - tuned[0], s[1] - tuned[1]), 0.02);
        assertTrue(what + " scored", VolleySettingsAccess.rate(v, want) >= 0.85);
      }
    }
  }

  @Test public void theParkVolleyUsesTheTunedParkOrTheSpotAtItsDistance() throws Exception {
    VolleySettings v = comp();
    for (Alliance a : new Alliance[] {Alliance.RED, Alliance.BLUE}) {
      for (Cell c : Cell.values()) {
        double[] p = AutoField.parkSpot(a, c);
        double[] aim = AutoField.autoAimPoint(a, c);
        // The near Cell's park is the tuned park; the far Cell's is 1.80 m out, nearest spot 9's 1.73 m
        boolean nearCell = Math.signum(aim[0]) == Math.signum(p[0]);
        assertEquals(a + " " + c, nearCell ? 10 : 9, v.spotFor(new FieldPose(p[0], p[1], p[2]), aim[0], aim[1]));
      }
    }
  }
}
