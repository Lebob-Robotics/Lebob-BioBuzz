package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.junit.Test;

public class VolleyTuningDataTest {
  @Test public void positionsResumeSeparatelyAndGlobalResetIsPreserved() {
    List<String> lines = Arrays.asList("session,mark,spot", "a,IN,0", "a,LONG,1", "b,RESET,-1", "b,IN,1");
    assertEquals(Arrays.asList(lines.get(0), lines.get(1), lines.get(3)), VolleyTuning.rowsForSpot(lines, 0));
    assertEquals(1, VolleyTuning.lastSpot(lines));
    assertEquals(0, VolleyTuning.lastSpot(lines.subList(0, 4)));
  }

  @Test public void startsAtTheSetSpotOrResumesTheLastOne() {
    List<String> lines = Arrays.asList("session,mark,spot", "a,IN,0", "a,LONG,1");
    assertEquals(4, VolleyTuning.startSpot(4, lines));
    assertEquals(1, VolleyTuning.startSpot(-1, lines));
    assertEquals(1, VolleyTuning.startSpot(99, lines)); // Off the grid: resume
  }

  @Test public void resetRowsMatchTheNewSchema() {
    String[] names = VolleyTuning.CSV_HEADER.split(",", -1);
    String[] cells = VolleyTuning.resetCsvRow("new").split(",", -1);
    assertEquals(names.length, cells.length);
    for (int i = 0; i < names.length; i++) {
      if (names[i].equals("mark")) { assertEquals("RESET", cells[i]); }
      if (names[i].equals("spot")) { assertEquals("-1", cells[i]); }
    }
  }

  @Test public void onlyNearbyConfirmedSpotsCanSupplyTeleOpSettings() {
    boolean[] known = new boolean[10]; known[0] = true;
    assertEquals(0, VolleySettings.nearestSpot(new Pose(-1, -0.32, 0), -0.44, -0.32, known));
    assertEquals(0, VolleySettings.nearestSpot(new Pose(1, 0.32, 0), 0.44, 0.32, known));
    assertEquals(-1, VolleySettings.nearestSpot(new Pose(1, 1, 0), -0.44, -0.32, known));
    known[0] = false;
    assertEquals(-1, VolleySettings.nearestSpot(new Pose(-1, -0.32, 0), -0.44, -0.32, known));
  }
  @Test public void actualWrittenRowsResumeTheSameAutomaticCandidate() {
    org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner tuner =
        new org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner();
    java.util.ArrayList<String> lines = new java.util.ArrayList<>();
    lines.add(VolleyTuning.CSV_HEADER);
    for (int v = 0; v < 8; v++) {
      org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner.Settings settings = tuner.current();
      org.firstinspires.ftc.teamcode.utils.VolleyCalibration c = tuner.calibration();
      c.startVolley(settings.firstTrim, settings.laterTrim);
      for (int i = 0; i < 4; i++) {
        c.fed();
        org.firstinspires.ftc.teamcode.utils.VolleyCalibration.Ball b = c.mark("BOUNCE");
        String row = VolleyTuning.resultRow("s", c, b, settings, 0, new double[]{-1, -.32}, 2, "four detected balls");
        assertEquals(VolleyTuning.CSV_HEADER.split(",", -1).length, row.split(",", -1).length);
        lines.add(row);
      }
      tuner.completed(c.balls(), 2, "four detected balls");
    }
    org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner restored =
        new org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner();
    restored.replay(VolleyTuning.rowsForSpot(lines, 0));
    assertEquals(2, tuner.current().profile); // 8 bounce volleys: two poor settings dropped after 3 each
    assertEquals(tuner.current().profile, restored.current().profile);
    assertEquals(tuner.current().firstTrim, restored.current().firstTrim, 0);
  }

}
