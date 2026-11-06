package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * The last Shot Tuning session, read back from its CSV so a restart carries on where it stopped: the session and
 * ball numbers, the spot and mode, the aim trims, and every ball with its last mark.
 */
public final class TuningResume {
  /**
   * One ball of the session, with the mark that counts (its last) and the one the search took (its first), each
   * null when unmarked.
   */
  public static final class Ball {
    public final int shot;
    public final boolean backward;
    public final int spot; // -1: not at a test spot
    public final double lipM;
    public final double xM;
    public final double yM;
    public final double rpm;
    public final double launchDeg;
    public final double trimDeg;
    public final String pass;
    public final String search;
    public String mark;
    public String firstMark;

    Ball(String[] c) {
      shot = Integer.parseInt(c[1]);
      backward = "BACKWARD".equals(c[3]);
      lipM = parse(c[4]);
      xM = parse(c[5]);
      yM = parse(c[6]);
      rpm = parse(c[13]);
      launchDeg = parse(c[15]);
      trimDeg = parse(c[18]);
      spot = c[19].startsWith("G") ? Integer.parseInt(c[19].substring(1)) : -1;
      pass = c.length > 21 ? c[21] : "";
      search = c.length > 22 ? c[22] : "";
    }

    /** Fired by a search, standing at its spot. */
    public boolean isSearchBall() {
      return pass.isEmpty() && (search.startsWith("hood:") || search.startsWith("rpm:"));
    }

    /** The search's phase as it was fired, e.g. FIND; empty for other balls. */
    public String phase() {
      int colon = search.indexOf(':');
      return isSearchBall() ? search.substring(colon + 1) : "";
    }

    /** "FRONT G3" or "BACK G3". */
    public String key() {
      return TuningResume.key(backward, spot);
    }
  }

  public final String session;
  public final List<Ball> balls; // In the order fired
  public final int lastShot;
  public final int scored;
  public final int bounced;
  public final boolean backward;
  public final int spot;
  public final double hoodedTrimDeg;
  public final double backwardTrimDeg;

  private TuningResume(String session, List<Ball> balls) {
    this.session = session;
    this.balls = Collections.unmodifiableList(balls);
    Ball last = balls.get(balls.size() - 1);
    int in = 0;
    int out = 0;
    int maxShot = 0;
    double hooded = 0;
    double back = 0;
    for (Ball b : balls) {
      in += "IN".equals(b.mark) ? 1 : 0;
      out += "BOUNCE".equals(b.mark) ? 1 : 0;
      maxShot = Math.max(maxShot, b.shot);
      if (b.backward) { back = b.trimDeg; } else { hooded = b.trimDeg; }
    }
    lastShot = maxShot;
    scored = in;
    bounced = out;
    backward = last.backward;
    spot = last.spot;
    hoodedTrimDeg = hooded;
    backwardTrimDeg = back;
  }

  /**
   * The spot after {@code current} among {@code 0..done.length-1} not yet done, wrapping round; {@code current}
   * itself only when it is the one left. -1 once all are done.
   */
  public static int nextSpot(int current, boolean[] done) {
    int count = done.length;
    int from = current >= 0 && current < count ? current : -1;
    for (int i = 1; i <= count; i++) {
      int candidate = ((from + i) % count + count) % count;
      if (!done[candidate]) {
        return candidate;
      }
    }
    return -1;
  }

  public static String key(boolean backward, int spot) {
    return (backward ? "BACK G" : "FRONT G") + spot;
  }

  /** The session of the CSV's last row, or null when the CSV has no balls. */
  public static TuningResume fromCsv(List<String> lines) {
    String session = null;
    for (int i = lines.size() - 1; i >= 0 && session == null; i--) {
      String[] c = lines.get(i).split(",", -1);
      if (c.length >= 20 && !"session".equals(c[0])) {
        session = c[0];
      }
    }
    if (session == null) {
      return null;
    }
    Map<Integer, Ball> byShot = new LinkedHashMap<>();
    for (String line : lines) {
      String[] c = line.split(",", -1);
      if (c.length < 20 || !session.equals(c[0])) {
        continue;
      }
      try {
        if ("SHOT".equals(c[2])) {
          byShot.put(Integer.parseInt(c[1]), new Ball(c));
        } else {
          Ball b = byShot.get(Integer.parseInt(c[1]));
          if (b != null) {
            b.mark = c[2];
            if (b.firstMark == null) {
              b.firstMark = c[2];
            }
          }
        }
      } catch (NumberFormatException e) {
        // A row cut short by a power cut: skip it
      }
    }
    return byShot.isEmpty() ? null : new TuningResume(session, new ArrayList<>(byShot.values()));
  }

  /**
   * The search balls at the last ball's spot and mode since that search started: each search begins in FIND,
   * and once a ball has gone in it never returns to FIND.
   */
  public List<Ball> currentSearchRun() {
    String key = key(backward, spot);
    List<Ball> run = new ArrayList<>();
    String lastPhase = null;
    for (Ball b : balls) {
      if (!b.isSearchBall() || !b.key().equals(key)) {
        continue;
      }
      if ("FIND".equals(b.phase()) && lastPhase != null && !"FIND".equals(lastPhase)) {
        run.clear();
      }
      run.add(b);
      lastPhase = b.phase();
    }
    return run;
  }

  /** The search balls of each spot and mode, in the order first fired. */
  public Map<String, List<Ball>> searchBallsBySpot() {
    Map<String, List<Ball>> map = new LinkedHashMap<>();
    for (Ball b : balls) {
      if (b.isSearchBall() && b.spot >= 0) {
        List<Ball> list = map.get(b.key());
        if (list == null) {
          list = new ArrayList<>();
          map.put(b.key(), list);
        }
        list.add(b);
      }
    }
    return map;
  }

  /** "centre (low-high) unit, in/fired in" from the scoring balls, as the live search shows it. */
  public static String summary(List<Ball> searchBalls) {
    boolean byRpm = searchBalls.get(0).search.startsWith("rpm:");
    double lo = Double.POSITIVE_INFINITY;
    double hi = Double.NEGATIVE_INFINITY;
    int in = 0;
    for (Ball b : searchBalls) {
      if ("IN".equals(b.mark)) {
        double v = byRpm ? b.rpm : b.launchDeg;
        lo = Math.min(lo, v);
        hi = Math.max(hi, v);
        in++;
      }
    }
    if (in == 0) {
      return String.format(Locale.US, "%d balls, none in", searchBalls.size());
    }
    return String.format(Locale.US, byRpm ? "%.0f (%.0f-%.0f) rpm, %d/%d in" : "%.1f (%.1f-%.1f) deg, %d/%d in",
        (lo + hi) / 2, lo, hi, in, searchBalls.size());
  }

  private static double parse(String s) {
    return s.isEmpty() ? Double.NaN : Double.parseDouble(s);
  }
}
