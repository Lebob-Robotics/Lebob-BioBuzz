package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class VolleyAutoTunerTest {
  private static void trial(VolleyAutoTuner tuner, String mark, double seconds) {
    VolleyAutoTuner.Settings s = tuner.current();
    VolleyCalibration c = tuner.calibration();
    c.startVolley(s.firstTrim, s.laterTrim);
    for (int i = 0; i < 4; i++) { c.fed().headingTrimDeg = s.headingTrim; c.mark(mark); }
    tuner.completed(c.balls(), seconds, "four detected balls");
  }

  @Test public void settingsAdvanceAutomaticallyAndNeverCertifyBounces() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    for (int p = 0; p < VolleyAutoTuner.profileCount(); p++) {
      assertEquals(p, t.current().profile);
      for (int i = 0; i < VolleyAutoTuner.MAX_TRIALS_PER_PROFILE; i++) { trial(t, "BOUNCE", 2); }
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

  @Test public void fourBallSettingNeedsThreeAdditionalCleanVolleysToConfirm() {
    VolleyAutoTuner t = new VolleyAutoTuner();
    // Fixed bounded alternatives are all explored; an IN alone never means calibrated.
    for (int p = 0; p < VolleyAutoTuner.profileCount(); p++) {
      assertEquals(p, t.current().profile);
      trial(t, "IN", 2 + p); trial(t, "IN", 2 + p);
    }
    assertFalse(t.done());
    for (int i = 0; i < VolleyAutoTuner.CONFIRM_VOLLEYS - 1; i++) { trial(t, "IN", 2); assertFalse(t.done()); }
    trial(t, "IN", 2);
    assertTrue(t.done());
    assertEquals(0, t.current().profile);
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
