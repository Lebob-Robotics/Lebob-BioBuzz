package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Automatic four-ball search. Operator supplies outcomes, never numeric settings. */
public final class VolleyAutoTuner {
  public static final int MAX_TRIALS_PER_PROFILE = 6;
  /** A volley is good with this many single (not doubled) balls IN; four in a row was too rare to ever finish. */
  public static final int GOOD_INS = 3;
  /** The best setting is confirmed by CONFIRM_GOOD good volleys within CONFIRM_ATTEMPTS. */
  public static final int CONFIRM_GOOD = 2;
  public static final int CONFIRM_ATTEMPTS = 3;
  /**
   * A setting is dropped early when its first POOR_TRIALS scored volleys all had at most POOR_INS IN and at least
   * POOR_BOUNCES bounce-outs. Short/long misses don't count: those say the trim is off, which the search fixes.
   */
  public static final int POOR_TRIALS = 3;
  public static final int POOR_INS = 1;
  public static final int POOR_BOUNCES = 2;
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
  private int goodHere;
  private int poorHere; // Scored volleys at this setting that were poor (see POOR_TRIALS)
  // Every scored volley's balls and single INs, per setting, for estimate()
  private final int[] scoredBalls = new int[PROFILES.length];
  private final int[] scoredIns = new int[PROFILES.length];
  private int timeoutsHere;
  private int confirmed;
  private int confirmTries;
  private int rounds = 1;
  private boolean confirming;
  private boolean done;
  private Settings best;
  private int bestIns;
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
    return new Settings(profile, trim(c.firstSearch(), c.bounceExplorations(true), c.searchStart(true)),
        trim(c.laterSearch(), c.bounceExplorations(false), c.searchStart(false)),
        Math.max(-8, Math.min(8, headings[profile].trimDeg())));
  }
  private static double trim(ShotSearch search, int bounces, double start) {
    // BOUNCE alone gives ShotSearch no direction before its first IN: explore both ways around the start.
    if (search.scored() == 0 && search.phase() == ShotSearch.Phase.FIND && search.next() == start) {
      double t = start + BOUNCE_SWEEP[Math.min(BOUNCE_SWEEP.length - 1, bounces)];
      return Math.max(-VolleyCalibration.SEARCH_LIMIT_DEG, Math.min(VolleyCalibration.SEARCH_LIMIT_DEG, t));
    }
    return search.next();
  }

  /**
   * Complete four-result trials drive the setting choice; unknown outcomes can never certify a candidate. (The trim
   * searches already learn from every seen ball as it is marked, including in volleys with unseen balls.)
   */
  public void completed(List<VolleyCalibration.Ball> balls, double seconds, String reason) {
    if (balls.size() != VolleyTrial.BALLS || !Double.isFinite(seconds) || seconds <= 0) { return; }
    boolean unseen = false;
    int ins = 0, bounces = 0;
    for (VolleyCalibration.Ball b : balls) {
      unseen |= b.mark == null || VolleyCalibration.UNSEEN.equals(b.mark);
      if ("IN".equals(b.mark) && !b.doubleBall(ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM)) { ins++; }
      if (b.mark != null && !VolleyCalibration.UNSEEN.equals(b.mark) && !VolleyCalibration.HIT.equals(b.mark)
          && BallMark.parse(b.mark).bounced) { bounces++; }
    }
    boolean good = ins >= GOOD_INS;
    if ("position lost".equals(reason) || "Circle cancelled".equals(reason)) {
      status = "Aborted trial: repeat this automatic setting"; return;
    }
    if (unseen) {
      // A setting that repeatedly cannot deliver four balls must not trap the automatic search.
      if (!done && !confirming && "timeout".equals(reason) && ++timeoutsHere >= 2) { advance(); }
      status = "Unseen balls are not scored; repeated delivery timeouts try the next automatic setting";
      return;
    }
    int scoredProfile = current().profile;
    scoredBalls[scoredProfile] += balls.size();
    scoredIns[scoredProfile] += ins;
    if (done) {
      if (good) { return; }
      // A miss after confirming starts a fresh confirmation; this volley is its first attempt.
      done = false; confirming = true; confirmed = confirmTries = 0;
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
      confirmTries++;
      if (good) { confirmed++; }
      if (confirmed >= CONFIRM_GOOD) {
        done = true;
        status = "CONFIRMED: " + CONFIRM_GOOD + " of " + confirmTries + " volleys had " + GOOD_INS + "+ IN";
      } else if (confirmTries - confirmed > CONFIRM_ATTEMPTS - CONFIRM_GOOD) {
        // Reopen the search instead of calling an unreliable configuration calibrated.
        confirming = false; confirmed = confirmTries = 0; profile = best.profile;
        trialsHere = goodHere = poorHere = timeoutsHere = 0;
        best = null; bestIns = 0; bestSeconds = Double.POSITIVE_INFINITY;
        status = "Confirmation missed: automatically reopening search";
      } else {
        status = "Confirming saved best setting";
      }
      return;
    }
    if (good && (ins > bestIns || (ins == bestIns && seconds < bestSeconds))) {
      best = fired; bestIns = ins; bestSeconds = seconds;
    }
    trialsHere++;
    goodHere = good ? goodHere + 1 : 0;
    if (ins <= POOR_INS && bounces >= POOR_BOUNCES) { poorHere++; }
    VolleyCalibration c = models[profile];
    boolean ranged = c.firstSearch().hasRange() && c.laterSearch().hasRange();
    boolean poor = trialsHere >= POOR_TRIALS && poorHere == trialsHere;
    status = poor ? "Setting scored poorly: moving on" : "Automatically searching the next trial";
    if (trialsHere >= MAX_TRIALS_PER_PROFILE || goodHere >= 2 || (good && ranged) || poor) { advance(); }
  }

  private void advance() {
    profile++; trialsHere = goodHere = poorHere = timeoutsHere = 0;
    if (profile >= PROFILES.length) {
      if (best != null) { confirming = true; confirmed = confirmTries = 0; }
      else { profile = 0; rounds++; }
    }
    // Confirming leaves profile past the end; a setting that already has balls ignores the seed.
    if (profile < PROFILES.length) { models[profile].seed(learnedCentre(profile, true), learnedCentre(profile, false)); }
  }

  /**
   * The scoring centre learned at the most similar other setting (feed rate matters most: a feed-0.3 trim sends
   * feed-0.8 balls long), or 0 when no setting has scored that ball yet.
   */
  private double learnedCentre(int target, boolean first) {
    boolean[] scored = new boolean[models.length];
    for (int i = 0; i < models.length; i++) {
      scored[i] = i != target && Double.isFinite((first ? models[i].firstSearch() : models[i].laterSearch()).centre());
    }
    int from = similarSetting(target, scored);
    if (from < 0) { return 0; }
    return (first ? models[from].firstSearch() : models[from].laterSearch()).centre();
  }

  /** The candidate setting closest to {@code target} in feeder, indexer, boost and recovery; -1 if none. */
  static int similarSetting(int target, boolean[] candidates) {
    int best = -1;
    double bestDistance = Double.POSITIVE_INFINITY;
    double[] t = PROFILES[target];
    for (int i = 0; i < PROFILES.length && i < candidates.length; i++) {
      if (!candidates[i] || i == target) { continue; }
      double[] p = PROFILES[i];
      double d = Math.abs(p[0] - t[0]) + Math.abs(p[1] - t[1]) / 2 + Math.abs(p[2] - t[2]) / 1000
          + Math.abs(p[3] - t[3]) * 0.05;
      if (d <= bestDistance) { best = i; bestDistance = d; } // Ties go to the later setting
    }
    return best;
  }

  public static int profileCount() { return PROFILES.length; }
  /** The best setting found so far, confirmed or not; null before any good volley (or after a failed confirm). */
  public Settings best() { return best; }
  /** Single INs in {@link #best}'s volley; 0 without one. */
  public int bestIns() { return best == null ? 0 : bestIns; }

  /**
   * The setting to shoot with now: the confirmed one once done; until then the setting whose scored volleys put
   * the most balls IN, (ins + 1) / (balls + 2), at the middle of the trims that scored there. One lucky volley
   * shouldn't pick the setting, and the trims fired in one volley are only one probe of the range. Null with no
   * scored volley yet.
   */
  public Settings estimate() {
    if (done) { return best; }
    int pick = -1;
    double pickRate = -1;
    for (int p = 0; p < PROFILES.length; p++) {
      if (scoredBalls[p] == 0) { continue; }
      double rate = (scoredIns[p] + 1.0) / (scoredBalls[p] + 2.0);
      if (rate > pickRate) { pick = p; pickRate = rate; }
    }
    if (pick < 0) { return null; }
    VolleyCalibration c = models[pick];
    return new Settings(pick, centreOrNext(c.firstSearch()), centreOrNext(c.laterSearch()),
        Math.max(-8, Math.min(8, headings[pick].trimDeg())));
  }

  /** The {@link #estimate}'s IN rate, as estimated; NaN without one. */
  public double estimateRate() {
    Settings e = estimate();
    if (e == null) { return Double.NaN; }
    return (scoredIns[e.profile] + 1.0) / (scoredBalls[e.profile] + 2.0);
  }

  private static double centreOrNext(ShotSearch search) {
    double centre = search.centre();
    return Double.isFinite(centre) ? centre : search.next();
  }
  public boolean done() { return done; }
  public int confirmed() { return confirmed; }
  public String status() {
    if (done || confirming) {
      return (done ? "DONE" : "CONFIRM " + confirmed + "/" + CONFIRM_GOOD + " good, try "
          + Math.min(confirmTries + 1, CONFIRM_ATTEMPTS) + "/" + CONFIRM_ATTEMPTS) + ": " + status;
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
    profile = trialsHere = goodHere = poorHere = timeoutsHere = confirmed = confirmTries = 0; rounds = 1;
    confirming = done = false;
    best = null; bestIns = 0; bestSeconds = Double.POSITIVE_INFINITY;
    Arrays.fill(scoredBalls, 0); Arrays.fill(scoredIns, 0);
  }

  private static String field(String[] c, Map<String, Integer> cols, String key) {
    Integer i = cols.get(key);
    return i != null && i < c.length ? c[i] : "";
  }
}
