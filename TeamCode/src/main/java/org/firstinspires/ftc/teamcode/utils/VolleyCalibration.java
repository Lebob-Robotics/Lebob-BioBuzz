package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Volley Tuning's bookkeeping, apart from the robot so it can be tested: the balls of the last volley, their marks,
 * a {@link ShotSearch} on the launch angle trim for the first ball and one for balls 2-4, and the feed power.
 *
 * <p>Each volley fires the searches' current trims (the first ball's, then the others'). After it the driver marks
 * each ball in the order it was fired: IN, BOUNCE or a miss as in Shot Tuning, {@link #HIT} for a ball that hit
 * another, or {@link #UNSEEN}. A collision, or a wheel dip deep enough to be two balls leaving together, lowers the
 * feed power one step, so the queue follows the ball ahead less tightly. Every mark is a CSV row ({@link #row}), and
 * {@link #replay} carries a restarted session on from them.
 */
public final class VolleyCalibration {
  public static final int BALLS_PER_VOLLEY = 4;
  public static final String HIT = "HIT";
  public static final String UNSEEN = "UNSEEN";
  public static final String RESET = "RESET"; // A row that starts the searches over
  public static final double FEED_POWER_STEP = 0.1;
  public static final double MIN_FEED_POWER = 0.3;
  public static final double MAX_FEED_POWER = 1.0;
  // The hood search, as Shot Tuning's: a higher launch angle sends a hooded ball shorter
  static final double SEARCH_STEP_DEG = 1.0;
  static final double SEARCH_RESOLUTION_DEG = 0.5;
  static final double SEARCH_QUANTUM_DEG = 0.1;
  static final double SEARCH_LIMIT_DEG = 4.0;
  static final int CONFIRM_BALLS = 3;

  public static final String CSV_HEADER = "session,volley,ball,group,mark,trimDeg,feedPower,boostRpm,fastRecovery,"
      + "fedRpm,targetRpm,leftRpm,dipRpm,doubleBall,lipM,towardMps,sidewaysMps,hoodDeg,battery,nextFeedPower";

  /** One ball of a volley, in the order fed. */
  public static final class Ball {
    public final int volley; // Counted from 1 in this session
    public final int index; // 0 for the first ball of its volley
    public final double trimDeg; // Launch angle trim it was fired with
    public final double feedPower; // Feed power it was fired with
    public double fedRpm = Double.NaN; // Wheel speed as its feed started
    public double targetRpm = Double.NaN;
    public double leftRpm = Double.NaN; // Highest speed during its feed: about what it left at
    public double dipRpm = Double.NaN; // How far the wheels fell after it
    public double lipM = Double.NaN;
    public double towardMps = Double.NaN;
    public double sidewaysMps = Double.NaN;
    public double hoodDeg = Double.NaN;
    public boolean detected = true; // False for a physical ball whose wheel dip was not observed.
    public double headingTrimDeg;
    public double batteryVolts = Double.NaN;
    public double elapsedS = Double.NaN;
    public String mark; // Null until marked

    Ball(int volley, int index, double trimDeg, double feedPower) {
      this.volley = volley;
      this.index = index;
      this.trimDeg = trimDeg;
      this.feedPower = feedPower;
    }

    public boolean first() {
      return index == 0;
    }

    /** The dip was deep enough to be this ball and the next leaving together. */
    public boolean doubleBall(double doubleDipRpm) {
      return dipRpm > doubleDipRpm;
    }
  }

  private final boolean adjustFeedPower;
  private ShotSearch firstSearch = newSearch();
  private ShotSearch laterSearch = newSearch();
  private double feedPower;
  private final List<Ball> balls = new ArrayList<>(); // The last volley's
  private int marked;
  private int volleys;
  private double volleyFirstTrimDeg;
  private double volleyLaterTrimDeg;
  private double volleyFeedPower;
  private boolean loweredThisVolley;
  private int hits;
  private int doubles;
  private int firstBounces, laterBounces;
  private double firstStart, laterStart;

  public VolleyCalibration(double feedPower) { this(feedPower, true); }

  /** Automatic profile search owns its feed power; legacy calibration lowers it after collisions. */
  public VolleyCalibration(double feedPower, boolean adjustFeedPower) {
    this.adjustFeedPower = adjustFeedPower;
    this.feedPower = clampPower(feedPower);
  }

  /** Starts both searches of an unused calibration at trims learned elsewhere; ignored once any ball is recorded. */
  public void seed(double firstTrimDeg, double laterTrimDeg) {
    if (firstSearch.balls() > 0 || laterSearch.balls() > 0 || firstBounces > 0 || laterBounces > 0) { return; }
    firstSearch = newSearch(firstTrimDeg);
    laterSearch = newSearch(laterTrimDeg);
    firstStart = firstSearch.next(); // Rounded and clamped as it will be fired
    laterStart = laterSearch.next();
  }

  /** Where that search started: 0 unless {@link #seed}ed. */
  public double searchStart(boolean first) { return first ? firstStart : laterStart; }

  static ShotSearch newSearch() { return newSearch(0); }

  static ShotSearch newSearch(double start) {
    return new ShotSearch(start, -1, SEARCH_STEP_DEG, SEARCH_RESOLUTION_DEG, SEARCH_QUANTUM_DEG, -SEARCH_LIMIT_DEG,
        SEARCH_LIMIT_DEG, CONFIRM_BALLS);
  }

  /** The trim to fire the next volley's first ball with. */
  public double nextFirstTrimDeg() { return firstSearch.next(); }

  /** The trim to fire the next volley's balls 2-4 with. */
  public double nextLaterTrimDeg() { return laterSearch.next(); }

  public ShotSearch firstSearch() { return firstSearch; }

  public ShotSearch laterSearch() { return laterSearch; }

  public double feedPower() { return feedPower; }

  public void setFeedPower(double power) { feedPower = clampPower(power); }

  public int volleys() { return volleys; }

  public int hits() { return hits; }

  public int doubles() { return doubles; }

  public int bounceExplorations(boolean first) { return first ? firstBounces : laterBounces; }

  private void recordSearch(boolean first, double trim, BallMark mark) {
    int count = first ? firstBounces : laterBounces;
    if (mark.bounced) { count++; }
    else if (mark.in || mark.hasRange()) { count = 0; }
    if (first) { firstBounces = count; } else { laterBounces = count; }
    (first ? firstSearch : laterSearch).record(trim, mark);
  }

  public List<Ball> balls() { return balls; }

  /** Balls of the last volley marked so far. */
  public int marked() { return marked; }

  /** Some ball of the last volley is still to be marked. */
  public boolean marksPending() { return marked < balls.size(); }

  /**
   * A volley starts with the trims it fires. Balls of the last one still unmarked are returned as {@link #UNSEEN},
   * for the CSV.
   */
  public List<Ball> startVolley() { return startVolley(nextFirstTrimDeg(), nextLaterTrimDeg()); }

  /** Freeze the settings actually fired, including automatic exploration/confirmation overrides. */
  public List<Ball> startVolley(double firstTrimDeg, double laterTrimDeg) {
    List<Ball> unseen = new ArrayList<>();
    while (marksPending()) {
      unseen.add(mark(UNSEEN));
    }
    balls.clear();
    marked = 0;
    volleys++;
    loweredThisVolley = false;
    volleyFirstTrimDeg = firstTrimDeg;
    volleyLaterTrimDeg = laterTrimDeg;
    volleyFeedPower = feedPower;
    return unseen;
  }

  /** The trim this volley fires ball {@code index} with. */
  public double volleyTrimDeg(int index) {
    return index == 0 ? volleyFirstTrimDeg : volleyLaterTrimDeg;
  }

  /** A ball's feed started; returns it, to fill in. */
  public Ball fed() {
    if (balls.size() >= BALLS_PER_VOLLEY) { throw new IllegalStateException("four-ball volley already full"); }
    Ball ball = new Ball(volleys, balls.size(), volleyTrimDeg(balls.size()), volleyFeedPower);
    balls.add(ball);
    return ball;
  }

  /** Reserve every physical ball for marking, including merged/missed dips and aborted trials. */
  public void finishVolley() {
    while (balls.size() < BALLS_PER_VOLLEY) { fed().detected = false; }
  }

  /**
   * The wheels' dip after the {@code dipIndex}th ball seen leaving (0 first), as it deepens. One past twice a ball's
   * lowers the feed power, once a volley.
   */
  public void dip(int dipIndex, double leftRpm, double dipRpm, double doubleDipRpm) {
    if (dipIndex < 0 || dipIndex >= balls.size()) {
      return;
    }
    Ball ball = balls.get(dipIndex);
    ball.leftRpm = leftRpm;
    boolean wasDouble = ball.doubleBall(doubleDipRpm);
    ball.dipRpm = Double.isNaN(ball.dipRpm) ? dipRpm : Math.max(ball.dipRpm, dipRpm);
    if (!wasDouble && ball.doubleBall(doubleDipRpm)) {
      doubles++;
      lowerFeedPower();
    }
  }

  /**
   * Marks the next ball of the last volley: a {@link BallMark}'s text, {@link #HIT} or {@link #UNSEEN}. A mark past
   * the balls the feed counted is a ball that went with another; it gets the later balls' trim. Returns the ball.
   */
  public Ball mark(String mark) {
    if (marked >= BALLS_PER_VOLLEY) { throw new IllegalStateException("all four balls already marked"); }
    Ball ball;
    if (marked < balls.size()) {
      ball = balls.get(marked);
    } else {
      ball = new Ball(volleys, balls.size(), volleyTrimDeg(balls.size()), volleyFeedPower);
      balls.add(ball);
    }
    marked++;
    ball.mark = mark;
    if (HIT.equals(mark)) {
      hits++;
      lowerFeedPower();
    } else if (!UNSEEN.equals(mark)) {
      recordSearch(ball.first(), ball.trimDeg, BallMark.parse(mark));
    }
    return ball;
  }

  /** The rest of the last volley's balls weren't seen; returns them, for the CSV. */
  public List<Ball> markRestUnseen() {
    List<Ball> unseen = new ArrayList<>();
    while (marksPending()) {
      unseen.add(mark(UNSEEN));
    }
    return unseen;
  }

  private void lowerFeedPower() {
    if (adjustFeedPower && !loweredThisVolley) {
      feedPower = clampPower(feedPower - FEED_POWER_STEP);
      loweredThisVolley = true;
    }
  }

  private static double clampPower(double power) {
    double p = Math.round(power * 100) / 100.0;
    return Math.max(MIN_FEED_POWER, Math.min(MAX_FEED_POWER, p));
  }

  /** The CSV row for a marked ball (columns as {@link #CSV_HEADER}). */
  public String row(String session, Ball ball, double boostRpm, boolean fastRecovery, double doubleDipRpm,
      double batteryVolts) {
    return String.format(Locale.US, "%s,%d,%d,%s,%s,%.1f,%.2f,%.0f,%b,%.0f,%.0f,%.0f,%.0f,%b,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
        session, ball.volley, ball.index + 1, ball.first() ? "FIRST" : "LATER", ball.mark, ball.trimDeg, ball.feedPower,
        boostRpm, fastRecovery, ball.fedRpm, ball.targetRpm, ball.leftRpm, ball.dipRpm,
        ball.doubleBall(doubleDipRpm), ball.lipM, ball.towardMps, ball.sidewaysMps, ball.hoodDeg, batteryVolts,
        feedPower);
  }

  /** The marker row that starts a session over. */
  public String resetRow(String session) {
    return session + ",0,0,," + RESET + ",,,,,,,,,,,,,,," + String.format(Locale.US, "%.2f", feedPower);
  }

  /**
   * Carries the searches and feed power on from a CSV's rows (header first). Rows after the last {@link #RESET} only.
   */
  public void replay(List<String> lines) {
    if (lines.isEmpty()) {
      return;
    }
    String[] names = lines.get(0).split(",", -1);
    int group = indexOf(names, "group");
    int mark = indexOf(names, "mark");
    int trim = indexOf(names, "trimDeg");
    int next = indexOf(names, "nextFeedPower");
    if (group < 0 || mark < 0 || trim < 0) {
      return;
    }
    for (String line : lines.subList(1, lines.size())) {
      String[] c = line.split(",", -1);
      if (c.length <= Math.max(group, Math.max(mark, trim))) {
        continue;
      }
      if (RESET.equals(c[mark])) {
        firstSearch = newSearch();
        laterSearch = newSearch();
        firstStart = laterStart = 0;
        hits = 0;
        doubles = 0;
        firstBounces = laterBounces = 0;
      } else if (HIT.equals(c[mark])) {
        hits++;
      } else if (!UNSEEN.equals(c[mark]) && !c[mark].isEmpty()) {
        try {
          double t = Double.parseDouble(c[trim]);
          recordSearch("FIRST".equals(c[group]), t, BallMark.parse(c[mark]));
        } catch (NumberFormatException ignored) {
          // A row cut short when the robot lost power
        }
      }
      if (next >= 0 && next < c.length && !c[next].isEmpty()) {
        try {
          feedPower = clampPower(Double.parseDouble(c[next]));
        } catch (NumberFormatException ignored) {
          // As above
        }
      }
    }
  }

  private static int indexOf(String[] names, String name) {
    for (int i = 0; i < names.length; i++) {
      if (names[i].equals(name)) {
        return i;
      }
    }
    return -1;
  }

  /** "FIND next -1.0" or "found -2.1..0.4, centre -0.9 (3/3 confirmed)". */
  public static String describe(ShotSearch search) {
    double[] range = search.range();
    if (search.hasRange()) {
      return String.format(Locale.US, "%s %.1f..%.1f deg, centre %+.1f  (confirm %d/%d in)",
          search.phase() == ShotSearch.Phase.DONE ? "FOUND" : "CONFIRM", range[0], range[1], search.centre(),
          search.confirmScored(), search.confirmFired());
    }
    return String.format(Locale.US, "%s  next %+.1f deg  (%d/%d in)", search.phase(), search.next(), search.scored(),
        search.balls());
  }
}
