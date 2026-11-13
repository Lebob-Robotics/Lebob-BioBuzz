package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class VolleyAutoTunerTest {
  private static void trial(VolleyAutoTuner tuner, String mark, double seconds) {
    trial(tuner, seconds, mark, mark, mark, mark);
  }

  private static void trial(VolleyAutoTuner tuner, double seconds, String... marks) {
    VolleyAutoTuner.Settings s = tuner.current();
    VolleyCalibration c = tuner.calibration();
    c.startVolley(s.firstTrim, s.laterTrim);
    for (String mark : marks) { c.fed().headingTrimDeg = s.headingTrim; c.mark(mark); }
    tuner.completed(c.balls(), seconds, "four detected balls");
  }

  /** Runs every setting with one good volley and fills the rest of its trials with bounces. */
  private static VolleyAutoTuner throughAllSettings(String... goodMarks) {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int p = 0; p < VolleyAutoTuner.profileCount(); p++) {
      assertEquals(p, t.current().profile);
      trial(t, 2 + p, goodMarks);
      for (int i = 1; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE; i++) { trial(t, "BOUNCE", 2); }
    }
    return t;
  }

  @Test public void settingsAdvanceAutomaticallyAndNeverCertifyBounces() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int p = 0; p < VolleyAutoTuner.profileCount(); p++) {
      assertEquals(p, t.current().profile);
      // All bounces: a poor setting is dropped after POOR_TRIALS volleys.
      for (int i = 0; i < VolleyAutoTuner.POOR_TRIALS; i++) { trial(t, "BOUNCE", 2); }
    }
    assertFalse(t.done());
    assertEquals(0, t.current().profile);
  }

  @Test public void firstBounceExploresHoodWithoutAnyManualAdjustment() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    assertEquals(0, t.current().firstTrim, 0);
    trial(t, "BOUNCE", 2);
    assertNotEquals(0, t.current().firstTrim, 0);
    assertNotEquals(0, t.current().laterTrim, 0);
  }

  @Test public void directionalMissStillMovesTheSearchInTheCorrectDirection() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, "SHORT", 2);
    assertTrue(t.current().firstTrim < 0);
    assertTrue(t.current().laterTrim < 0);
  }

  @Test public void bestSettingNeedsTwoGoodVolleysOutOfThreeToConfirm() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    // Fixed bounded alternatives are all explored; an IN alone never means calibrated.
    for (int p = 0; p < VolleyAutoTuner.profileCount(); p++) {
      assertEquals(p, t.current().profile);
      trial(t, "IN", 2 + p); trial(t, "IN", 2 + p);
    }
    assertFalse(t.done());
    trial(t, "IN", 2); assertFalse(t.done());
    trial(t, 2, "IN", "BOUNCE", "SHORT", "IN"); assertFalse(t.done()); // One miss is allowed
    trial(t, 2, "IN", "IN", "LONG", "IN");
    assertTrue(t.done());
    assertEquals(0, t.current().profile);
  }

  @Test public void threeOfFourInCountsAsAGoodVolley() {
    VolleyAutoTuner t = throughAllSettings("IN", "IN", "BOUNCE", "IN");
    assertTrue(t.status().startsWith("CONFIRM"));
  }

  @Test public void twoOfFourInIsNotGood() {
    VolleyAutoTuner t = throughAllSettings("IN", "IN", "BOUNCE", "SHORT");
    assertFalse(t.status().startsWith("CONFIRM"));
    assertEquals(0, t.current().profile);
  }

  @Test public void bestPrefersMoreInsOverSpeed() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, 2, "IN", "IN", "BOUNCE", "IN"); // Setting 1: three IN, fast
    for (int i = 1; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE; i++) { trial(t, "BOUNCE", 2); }
    trial(t, 9, "IN", "IN", "IN", "IN"); // Setting 2: four IN, slow
    for (int i = 1; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE; i++) { trial(t, "BOUNCE", 2); }
    for (int p = 2; p < VolleyAutoTuner.profileCount(); p++) {
      for (int i = 0; i < VolleyAutoTuner.POOR_TRIALS; i++) { trial(t, "BOUNCE", 2); }
    }
    assertTrue(t.status().startsWith("CONFIRM"));
    assertEquals(1, t.current().profile);
  }

  @Test public void twoConfirmationMissesReopenTheSearch() {
    VolleyAutoTuner t = throughAllSettings("IN", "IN", "IN", "IN");
    trial(t, "BOUNCE", 2);
    assertTrue(t.status().startsWith("CONFIRM"));
    trial(t, "BOUNCE", 2);
    assertFalse(t.status().startsWith("CONFIRM"));
    assertFalse(t.done());
  }

  @Test public void nextSettingStartsFromTheTrimLearnedAtTheLastOne() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    // Push setting 1's searches short until they score low, then let it run out of trials.
    trial(t, "SHORT", 2);
    assertTrue(t.current().firstTrim < 0 && t.current().laterTrim < 0);
    trial(t, "IN", 2);
    double first = t.calibration().firstSearch().centre(), later = t.calibration().laterSearch().centre();
    assertTrue(first < 0 && later < 0);
    // Collision marks use up the trials without moving the searches.
    for (int i = 2; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE; i++) { trial(t, VolleyCalibration.HIT, 2); }
    assertEquals(1, t.current().profile);
    assertEquals(first, t.current().firstTrim, 1e-9);
    assertEquals(later, t.current().laterTrim, 1e-9);
    trial(t, "BOUNCE", 2); // A bounce with no IN yet explores around the learned start, not around 0
    assertEquals(first + 1, t.current().firstTrim, 1e-9);
  }

  @Test public void poorSettingIsDroppedAfterThreeVolleys() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, 2, "IN", "BOUNCE", "BOUNCE", "SHORT");
    trial(t, "BOUNCE", 2);
    assertEquals(0, t.current().profile);
    trial(t, 2, "BOUNCE", "LONG", "IN", "BOUNCE");
    assertEquals(1, t.current().profile);
  }

  @Test public void shortAndLongMissesDoNotDropASetting() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, 2, "LONG", "SHORT", "BOUNCE", "SHORT");
    trial(t, 2, "SHORT", "LONG", "LONG", "LONG");
    trial(t, 2, "IN", "SHORT", "LONG", "LONG");
    assertEquals(0, t.current().profile);
  }

  @Test public void newSettingStartsFromTheMostSimilarFeed() {
    boolean[] tried = new boolean[VolleyAutoTuner.profileCount()];
    for (int i = 0; i <= 5; i++) { tried[i] = true; }
    // Setting 7 (feed 0.8, indexer 0.75, boost 100) learns from setting 3 (0.8, 0.75), not from setting 6 (0.3).
    assertEquals(2, VolleyAutoTuner.similarSetting(6, tried));
    tried[2] = false;
    assertEquals(3, VolleyAutoTuner.similarSetting(6, tried)); // Then the same feeder (0.8/0.5) over 1.0/0.75
    assertEquals(-1, VolleyAutoTuner.similarSetting(0, new boolean[VolleyAutoTuner.profileCount()]));
  }

  @Test public void settingWithTwoInVolleyIsNotDroppedEarly() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, 2, "IN", "IN", "BOUNCE", "SHORT");
    for (int i = 1; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE - 1; i++) { trial(t, "BOUNCE", 2); }
    assertEquals(0, t.current().profile);
    trial(t, "BOUNCE", 2);
    assertEquals(1, t.current().profile);
  }

  @Test public void estimatePicksTheSettingWithTheBestInRateNotOneLuckyVolley() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    assertNull(t.estimate());
    trial(t, 1, "IN", "IN", "IN", "IN"); // Setting 1: one perfect volley, fastest
    for (int i = 0; i < 2; i++) { trial(t, 2, "BOUNCE", "BOUNCE", "BOUNCE", "IN"); } // ...then it bounces
    trial(t, 2, "BOUNCE", "SHORT", "LONG", "SHORT");
    trial(t, 2, "BOUNCE", "BOUNCE", "BOUNCE", "BOUNCE");
    trial(t, 2, "BOUNCE", "BOUNCE", "BOUNCE", "BOUNCE");
    assertEquals(1, t.current().profile);
    // Setting 2: slower, but 3 of 4 each time (two good in a row then end it)
    for (int i = 0; i < 2; i++) { trial(t, 5, "IN", "IN", "BOUNCE", "IN"); }
    assertEquals(0, t.best().profile); // The search's best is still the fastest four-IN volley
    assertEquals(1, t.estimate().profile); // The estimate goes on the IN rate
    assertEquals((6 + 1.0) / (8 + 2.0), t.estimateRate(), 1e-9);
  }

  @Test public void seededProgressReplaysIdentically() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    List<String> lines = new ArrayList<>();
    lines.add(VolleyCalibration.CSV_HEADER + ",profile,trialSeconds,releaseReason");
    String[][] volleys = {{"SHORT", "SHORT", "SHORT", "SHORT"}, {"IN", "IN", "LONG", "IN"}};
    for (int v = 0; v < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE + 3; v++) {
      String[] marks = v < volleys.length ? volleys[v] : new String[] {"BOUNCE", "LONG", "IN", "SHORT"};
      VolleyAutoTuner.Settings settings = t.current();
      VolleyCalibration c = t.calibration();
      c.startVolley(settings.firstTrim, settings.laterTrim);
      for (String m : marks) {
        c.fed();
        VolleyCalibration.Ball b = c.mark(m);
        lines.add(c.row("s", b, settings.boostRpm, settings.fastRecovery, 600, 12)
            + "," + settings.profile + ",2.0,four detected balls");
      }
      t.completed(c.balls(), 2, "four detected balls");
    }
    VolleyAutoTuner restored = new VolleyAutoTuner(); restored.replay(lines);
    assertEquals(1, t.current().profile); // One bounce per volley is not poor: setting 2 keeps searching
    assertEquals(t.status(), restored.status());
    assertEquals(t.current().firstTrim, restored.current().firstTrim, 0);
    assertEquals(t.current().laterTrim, restored.current().laterTrim, 0);
  }

  @Test public void unseenOrAbortedTrialsDoNotAdvanceOrConfirm() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int i = 0; i < 20; i++) { trial(t, VolleyCalibration.UNSEEN, 12); }
    assertEquals(0, t.current().profile);
    assertFalse(t.done());
  }

  @Test public void profileFeedPowerIsFrozenEvenAfterCollisionMarks() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, VolleyCalibration.HIT, 2);
    assertEquals(t.current().feeder, t.calibration().feedPower(), 0);
  }

  @Test public void replayRestoresTheSameAutomaticProgressAndConfirmedResult() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    List<String> lines = new ArrayList<>();
    String header = VolleyCalibration.CSV_HEADER + ",profile,trialSeconds,releaseReason";
    lines.add(header);
    for (int volley = 0; volley < VolleyAutoTuner.profileCount() * VolleyAutoTuner.MAX_TRIALS_PER_PROFILE + 3; volley++) {
      VolleyAutoTuner.Settings settings = t.current();
      VolleyCalibration c = t.calibration();
      c.startVolley(settings.firstTrim, settings.laterTrim);
      for (int i = 0; i < 4; i++) {
        c.fed();
        VolleyCalibration.Ball b = c.mark("IN");
        lines.add(c.row("session", b, settings.boostRpm, settings.fastRecovery, 600, 12)
            + "," + settings.profile + ",2.0,four detected balls");
      }
      t.completed(c.balls(), 2, "four detected balls");
    }
    VolleyAutoTuner restored = new VolleyAutoTuner(); restored.replay(lines);
    assertTrue(t.done()); assertTrue(restored.done());
    assertEquals(t.current().profile, restored.current().profile);
    assertEquals(t.current().firstTrim, restored.current().firstTrim, 0);
    assertEquals(t.current().laterTrim, restored.current().laterTrim, 0);
  }
  @Test public void repeatedDeliveryTimeoutsTryAnotherSettingWithoutOperatorAdjustment() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int n = 0; n < 2; n++) {
      VolleyCalibration c = t.calibration();
      c.startVolley(); c.finishVolley();
      for (int i = 0; i < 4; i++) { c.mark(VolleyCalibration.UNSEEN); }
      t.completed(c.balls(), 12, "timeout");
    }
    assertEquals(1, t.current().profile);
    assertFalse(t.done());
  }

  @Test public void laterObservedFailureReopensEvenAConfirmedSetting() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int i = 0; i < 2 * VolleyAutoTuner.profileCount() + 3; i++) { trial(t, "IN", 2); }
    assertTrue(t.done());
    trial(t, "BOUNCE", 2);
    assertFalse(t.done());
  }

  @Test public void leftRightMarksAutomaticallyCorrectAim() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    trial(t, "LEFT", 2);
    assertTrue(t.current().headingTrim < 0);
    assertEquals(0, t.current().firstTrim, 0);
    assertEquals(0, t.current().laterTrim, 0);
    double before = t.current().headingTrim;
    trial(t, "RIGHT", 2);
    assertTrue(t.current().headingTrim > before);
    for (int i = 0; i < 30; i++) { trial(t, "LEFT", 2); }
    assertTrue(Math.abs(t.current().headingTrim) <= 8);
  }

}
