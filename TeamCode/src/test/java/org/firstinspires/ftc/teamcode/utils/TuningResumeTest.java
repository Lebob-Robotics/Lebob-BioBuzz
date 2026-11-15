package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.Test;

public class TuningResumeTest {
  private static final String HEADER = "session,shot,kind,mode,lip_m,x_m,y_m,heading_deg,cell_x_m,cell_y_m,"
      + "head_err_deg,toward_mps,sideways_mps,target_rpm,rpm_at_feed,launch_deg,hood_at_feed_deg,battery_v,"
      + "head_trim_deg,spot,rpm_dip,pass,search";

  private static String row(String session, int shot, String kind, String spot, double launch, double trim,
      String pass, String search) {
    return String.format(java.util.Locale.US,
        "%s,%d,%s,HOODED,0.800,-1.000,0.450,126.0,-0.440,-0.324,0.30,0.000,0.000,3330,3330,%.2f,%.2f,12.00,%.2f,%s,"
            + "300,%s,%s", session, shot, kind, launch, launch, trim, spot, pass, search);
  }

  private static void ball(List<String> lines, String session, int shot, String spot, double launch, String search,
      String... marks) {
    lines.add(row(session, shot, "SHOT", spot, launch, 0.5, "", search));
    for (String m : marks) {
      lines.add(row(session, shot, m, spot, launch, 0.5, "", search));
    }
  }

  @Test
  public void noBallsMeansNothingToCarryOn() {
    assertNull(TuningResume.fromCsv(new ArrayList<String>()));
    assertNull(TuningResume.fromCsv(Arrays.asList(HEADER)));
  }

  @Test
  public void carriesOnTheLastSessionWithItsLastMarks() {
    List<String> lines = new ArrayList<>();
    lines.add(HEADER);
    ball(lines, "OLD", 1, "G0", 60, "hood:FIND", "IN");
    ball(lines, "NEW", 1, "G4", 60, "hood:FIND", "IN");
    ball(lines, "NEW", 2, "G4", 58, "hood:LONG_EDGE", "SHORT", "BOUNCE"); // Remarked: the last counts
    ball(lines, "NEW", 3, "G5", 63, "hood:FIND");
    TuningResume r = TuningResume.fromCsv(lines);
    assertEquals("NEW", r.session);
    assertEquals(3, r.lastShot);
    assertEquals(3, r.balls.size());
    assertEquals(1, r.scored);
    assertEquals(1, r.bounced);
    assertEquals(5, r.spot);
    assertEquals(false, r.backward);
    assertEquals(0.5, r.hoodedTrimDeg, 1e-9);
    assertEquals("BOUNCE", r.balls.get(1).mark);
    assertEquals("SHORT", r.balls.get(1).firstMark);
    assertNull(r.balls.get(2).mark);
  }

  @Test
  public void theCurrentSearchStartsAtItsLastFind() {
    List<String> lines = new ArrayList<>();
    lines.add(HEADER);
    ball(lines, "S", 1, "G5", 63, "hood:FIND", "SHORT");
    ball(lines, "S", 2, "G5", 61, "hood:FIND", "IN");
    ball(lines, "S", 3, "G5", 59, "hood:LONG_EDGE", "SHORT");
    ball(lines, "S", 4, "G5", 59, "manual", "SHORT"); // Set by hand: not the search's
    ball(lines, "S", 5, "G5", 63, "hood:FIND", "IN"); // Restarted
    ball(lines, "S", 6, "G5", 61, "hood:LONG_EDGE", "LONG");
    ball(lines, "S", 7, "G5", 62, "hood:LONG_EDGE", "IN");
    List<TuningResume.Ball> run = TuningResume.fromCsv(lines).currentSearchRun();
    assertEquals(3, run.size());
    assertEquals(5, run.get(0).shot);
    assertEquals(7, run.get(2).shot);
  }

  @Test
  public void replayingTheRunRestoresTheSearch() {
    // The hooded shot reaches further with a lower launch angle
    ShotSearch live = new ShotSearch(63, -1, 2, 0.5, 0.1, 30, 99, 3);
    List<String> lines = new ArrayList<>();
    lines.add(HEADER);
    String[] marks = {"SHORT", "IN", "LONG", "IN"};
    for (int i = 0; i < marks.length; i++) {
      double fired = live.next();
      ball(lines, "S", i + 1, "G5", fired, "hood:" + live.phase().name(), marks[i]);
      live.record(fired, BallMark.parse(marks[i]));
    }
    List<TuningResume.Ball> run = TuningResume.fromCsv(lines).currentSearchRun();
    ShotSearch replayed = new ShotSearch(run.get(0).launchDeg, -1, 2, 0.5, 0.1, 30, 99, 3);
    for (TuningResume.Ball b : run) {
      replayed.record(b.launchDeg, BallMark.parse(b.mark));
    }
    assertEquals(live.phase(), replayed.phase());
    assertEquals(live.next(), replayed.next(), 1e-9);
    assertEquals(live.balls(), replayed.balls());
  }

  @Test
  public void nextSpotSkipsTheDoneOnesAndWraps() {
    boolean[] done = {true, true, false, true, true, false, true};
    assertEquals(5, TuningResume.nextSpot(2, done));
    assertEquals(2, TuningResume.nextSpot(5, done));
    assertEquals(2, TuningResume.nextSpot(-1, done));
    assertEquals(2, TuningResume.nextSpot(9, done)); // A front spot past the backward shot's spots
    assertEquals(2, TuningResume.nextSpot(2, new boolean[] {true, true, false})); // Only itself left
    assertEquals(-1, TuningResume.nextSpot(0, new boolean[] {true, true}));
  }

  @Test
  public void marksParseBackToWhatTheyWrote() {
    for (BallMark m : new BallMark[] {BallMark.IN, BallMark.BOUNCE, BallMark.miss(true, false, true, false),
        BallMark.miss(false, true, false, true), BallMark.miss(false, false, false, false)}) {
      assertEquals(m, BallMark.parse(m.toString()));
    }
  }
}
