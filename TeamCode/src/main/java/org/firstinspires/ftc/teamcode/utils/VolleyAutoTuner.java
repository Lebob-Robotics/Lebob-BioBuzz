package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Automatic four-ball search. Operator supplies outcomes, never numeric settings. */
public final class VolleyAutoTuner {
  public static final int MAX_TRIALS_PER_PROFILE = 8;
  public static final int CONFIRM_VOLLEYS = 3;
  private static final double[] BOUNCE_SWEEP = {0, 1, -1, 2, -2, 3, -3, 4, -4};
  // Bounded, measured alternatives: pressure first, then feed rate, boost, and PID recovery.
  private static final double[][] PROFILES = {
      {1, 1, 0, 1}, {1, .75, 0, 1}, {.8, .75, 0, 1}, {.8, .5, 0, 1},
      {.6, .5, 0, 1}, {.3, .5, 0, 1}, {.8, .75, 100, 1}, {.8, .75, 200, 1},
      {.8, .75, 300, 1}, {.8, .75, 0, 0}
  };

  public static final class Settings {
    public final int profile;
    public final double feeder, indexerScale, boostRpm, firstTrim, laterTrim, headingTrim;
    public final boolean fastRecovery;
    Settings(int profile, double firstTrim, double laterTrim, double headingTrim) {
      this.profile = profile;
      feeder = PROFILES[profile][0]; indexerScale = PROFILES[profile][1];
      boostRpm = PROFILES[profile][2]; fastRecovery = PROFILES[profile][3] != 0;
      this.firstTrim = firstTrim; this.laterTrim = laterTrim; this.headingTrim = headingTrim;
    }
  }

  private final VolleyCalibration[] models = new VolleyCalibration[PROFILES.length];
  private final AimTrimSearch[] headings = new AimTrimSearch[PROFILES.length];
  private int profile;
  private int trialsHere;
  private int cleanHere;
  private int timeoutsHere;
  private int confirmed;
  private int rounds = 1;
  private boolean confirming;
  private boolean done;
  private Settings best;
  private double bestSeconds = Double.POSITIVE_INFINITY;
  private String status = "";

  public VolleyAutoTuner() {
    for (int i = 0; i < models.length; i++) {
      models[i] = new VolleyCalibration(PROFILES[i][0], false);
      headings[i] = new AimTrimSearch(1, 0.25);
    }
  }

  public VolleyCalibration calibration() { return models[current().profile]; }
  public Settings current() {
    if (confirming || done) { return best; }
    VolleyCalibration c = models[profile];
    return new Settings(profile, trim(c.firstSearch(), c.bounceExplorations(true)), trim(c.laterSearch(), c.bounceExplorations(false)),
        Math.max(-8, Math.min(8, headings[profile].trimDeg())));
  }
  private static double trim(ShotSearch search, int bounces) {
    // BOUNCE alone gives ShotSearch no direction before its first IN: explore both ways automatically.
    if (search.scored() == 0 && search.phase() == ShotSearch.Phase.FIND && search.next() == 0) {
      return BOUNCE_SWEEP[Math.min(BOUNCE_SWEEP.length - 1, bounces)];
    }
    return search.next();
  }

  /** Complete four-result trials drive the search; unknown outcomes can never certify a candidate. */
  public void completed(List<VolleyCalibration.Ball> balls, double seconds, String reason) {
    if (balls.size() != VolleyTrial.BALLS || !Double.isFinite(seconds) || seconds <= 0) { return; }
    boolean unseen = false, clean = true;
    for (VolleyCalibration.Ball b : balls) {
      unseen |= b.mark == null || VolleyCalibration.UNSEEN.equals(b.mark);
      clean &= "IN".equals(b.mark) && !b.doubleBall(ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM);
    }
    if ("position lost".equals(reason) || "Circle cancelled".equals(reason)) {
      status = "Aborted trial: repeat this automatic setting"; return;
    }
    if (unseen) {
      // A setting that repeatedly cannot deliver four balls must not trap the automatic search.
      if (!done && !confirming && "timeout".equals(reason) && ++timeoutsHere >= 2) { advance(); }
      status = "Unseen balls are not scored; repeated delivery timeouts try the next automatic setting";
      return;
    }
    if (done) {
      if (clean) { return; }
      done = false; confirming = true;
    }
    int firedProfile = current().profile;
    Settings fired = new Settings(firedProfile, balls.get(0).trimDeg, balls.get(1).trimDeg,
        balls.get(0).headingTrimDeg);
    for (VolleyCalibration.Ball b : balls) {
      if (!VolleyCalibration.HIT.equals(b.mark)) {
        headings[firedProfile].record(BallMark.parse(b.mark), b.headingTrimDeg);
      }
    }
    if (confirming) {
      if (clean) {
        confirmed++;
        done = confirmed >= CONFIRM_VOLLEYS;
        status = done ? "CONFIRMED: three four-ball volleys retained" : "Confirming saved best setting";
      } else {
        // Reopen the search instead of calling a bouncing configuration calibrated.
        confirming = false; confirmed = 0; profile = best.profile; trialsHere = cleanHere = timeoutsHere = 0;
        best = null; bestSeconds = Double.POSITIVE_INFINITY;
        status = "Confirmation missed: automatically reopening search";
      }
      return;
    }
    if (clean && Double.isFinite(seconds) && seconds > 0 && seconds < bestSeconds) {
      best = fired; bestSeconds = seconds;
    }
    trialsHere++;
    cleanHere = clean ? cleanHere + 1 : 0;
    VolleyCalibration c = models[profile];
    boolean ranged = c.firstSearch().hasRange() && c.laterSearch().hasRange();
    if (trialsHere >= MAX_TRIALS_PER_PROFILE || cleanHere >= 2 || (clean && ranged)) { advance(); }
    status = confirming ? "Confirming fastest clean four-ball setting" : "Automatically searching the next trial";
  }

  private void advance() {
    profile++; trialsHere = cleanHere = timeoutsHere = 0;
    if (profile >= PROFILES.length) {
      if (best != null) { confirming = true; confirmed = 0; }
      else { profile = 0; rounds++; }
    }
  }

  public static int profileCount() { return PROFILES.length; }
  public boolean done() { return done; }
  public int confirmed() { return confirmed; }
  public String status() {
    if (done || confirming) {
      return (done ? "DONE" : "CONFIRM " + confirmed + "/" + CONFIRM_VOLLEYS) + ": " + status;
    }
    return "AUTO round " + rounds + ", setting " + (profile + 1) + "/" + PROFILES.length
        + ", trial " + (trialsHere + 1) + "/" + MAX_TRIALS_PER_PROFILE + ": " + status;
  }

  /** Replays marked rows, including candidate progression and final confirmation, after a restart. */
  public void replay(List<String> lines) {
    if (lines.isEmpty()) { return; }
    String header = lines.get(0);
    Map<String, Integer> cols = new HashMap<>();
    String[] names = header.split(",", -1);
    for (int i = 0; i < names.length; i++) { cols.put(names[i], i); }
    VolleyCalibration.Ball[] group = new VolleyCalibration.Ball[VolleyTrial.BALLS];
    String groupKey = "";
    for (String row : lines.subList(1, lines.size())) {
      String[] c = row.split(",", -1);
      try {
        String mark = field(c, cols, "mark");
        if (VolleyCalibration.RESET.equals(mark)) {
          reset(); Arrays.fill(group, null); groupKey = ""; continue;
        }
        int id = Integer.parseInt(field(c, cols, "profile"));
        int volley = Integer.parseInt(field(c, cols, "volley"));
        int index = Integer.parseInt(field(c, cols, "ball")) - 1;
        if (id < 0 || id >= models.length || index < 0 || index >= group.length || id != current().profile) { continue; }
        String key = field(c, cols, "session") + ":" + volley + ":" + id;
        if (!key.equals(groupKey)) { Arrays.fill(group, null); groupKey = key; }
        double trim = Double.parseDouble(field(c, cols, "trimDeg"));
        if (!Double.isFinite(trim) || Math.abs(trim) > 4.0) { continue; }
        models[id].replay(Arrays.asList(header, row));
        VolleyCalibration.Ball b = new VolleyCalibration.Ball(volley, index, trim, PROFILES[id][0]);
        b.mark = mark;
        String heading = field(c, cols, "headingTrimDeg");
        b.headingTrimDeg = heading.isEmpty() ? 0 : Double.parseDouble(heading);
        if (!Double.isFinite(b.headingTrimDeg) || Math.abs(b.headingTrimDeg) > 8) { continue; }
        b.dipRpm = Double.parseDouble(field(c, cols, "dipRpm"));
        group[index] = b;
        boolean complete = true;
        for (VolleyCalibration.Ball ball : group) { complete &= ball != null; }
        if (complete) {
          completed(new ArrayList<>(Arrays.asList(group)),
              Double.parseDouble(field(c, cols, "trialSeconds")), field(c, cols, "releaseReason"));
          Arrays.fill(group, null);
        }
      } catch (NumberFormatException ignored) {
        // A row cut short during power loss cannot change a setting or certify a trial.
      }
    }
  }

  private void reset() {
    for (int i = 0; i < models.length; i++) { models[i] = new VolleyCalibration(PROFILES[i][0], false);
      headings[i] = new AimTrimSearch(1, 0.25); }
    profile = trialsHere = cleanHere = timeoutsHere = confirmed = 0; rounds = 1; confirming = done = false;
    best = null; bestSeconds = Double.POSITIVE_INFINITY;
  }

  private static String field(String[] c, Map<String, Integer> cols, String key) {
    Integer i = cols.get(key);
    return i != null && i < c.length ? c[i] : "";
  }
}
