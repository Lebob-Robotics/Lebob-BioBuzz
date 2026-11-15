package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the range of one shot setting, the launch angle or the RPM, that scores from one spot. Each marked ball moves
 * the setting: from the first value until a ball goes in, toward the miss's opposite; then out past the highest
 * scoring value until a ball goes long and below the lowest until one goes short, halving the gap each ball until
 * both edges are known to {@code resolution}. Then a few balls at the middle of the range confirm it.
 *
 * <p>Works in "reach", the setting times {@code longerSign}, so that more reach always sends the ball further.
 * Once a ball has gone in, a miss fired past the scoring values marks that side's edge whatever it was called (a
 * short fired past the longest scoring ball is counted as {@link #contrary}), and a bounce-out counts by which side
 * of the scoring values it was fired on. A ball marked only left or right, or both short and long, says nothing
 * about range and the same value is fired again.
 */
public final class ShotSearch {
  public enum Phase { FIND, LONG_EDGE, SHORT_EDGE, CONFIRM, DONE }

  private final double longerSign;
  private final double step;
  private final double resolution;
  private final double quantum;
  private final double minReach;
  private final double maxReach;
  private final int confirmBalls;

  private final List<Double> scoredReach = new ArrayList<>();
  private final List<double[]> confirmMisses = new ArrayList<>(); // {reach, 1 if long} of confirming range misses
  private double shortestLongReach = Double.POSITIVE_INFINITY; // Least reach that went long
  private double longestShortReach = Double.NEGATIVE_INFINITY; // Most reach that went short
  private double nextReach;
  private Phase phase = Phase.FIND;
  private int balls;
  private int scored;
  private int confirmFired;
  private int confirmScored;
  private int contrary;

  /**
   * @param start the setting to fire first, usually the table's
   * @param longerSign +1 when raising the setting sends the ball further, -1 when lowering it does
   * @param step how far to move while no ball has gone in, or past a scoring edge not yet bracketed
   * @param resolution the edges are found to within this
   * @param quantum settings are rounded to a multiple of this
   * @param min lowest setting allowed
   * @param max highest setting allowed
   * @param confirmBalls balls fired at the middle of the range once both edges are known
   */
  public ShotSearch(double start, double longerSign, double step, double resolution, double quantum,
      double min, double max, int confirmBalls) {
    this.longerSign = Math.signum(longerSign) == 0 ? 1 : Math.signum(longerSign);
    this.step = step;
    this.resolution = resolution;
    this.quantum = quantum;
    this.minReach = Math.min(this.longerSign * min, this.longerSign * max);
    this.maxReach = Math.max(this.longerSign * min, this.longerSign * max);
    this.confirmBalls = confirmBalls;
    nextReach = clamp(this.longerSign * start);
  }

  /** The setting to fire next. */
  public double next() {
    return round(nextReach / longerSign);
  }

  public Phase phase() { return phase; }
  public int balls() { return balls; }
  public int scored() { return scored; }
  public int confirmFired() { return confirmFired; }
  public int confirmScored() { return confirmScored; }

  /** Misses past the scoring values marked the other way: a short where it should go long, or a long where short. */
  public int contrary() { return contrary; }

  /** The furthest-reaching setting marked short (whatever was marked, past the scoring values), NaN if none. */
  public double furthestShort() {
    return longestShortReach == Double.NEGATIVE_INFINITY ? Double.NaN : longestShortReach / longerSign;
  }

  /** The least-reaching setting marked long, NaN if none. */
  public double nearestLong() {
    return shortestLongReach == Double.POSITIVE_INFINITY ? Double.NaN : shortestLongReach / longerSign;
  }

  /** True once both edges are known. */
  public boolean hasRange() {
    return phase == Phase.CONFIRM || phase == Phase.DONE;
  }

  /** The lowest and highest setting that scores, as far as the balls so far show; NaN until a ball goes in. */
  public double[] range() {
    if (scoredReach.isEmpty()) {
      return new double[] {Double.NaN, Double.NaN};
    }
    double a = round(lowerEdgeReach() / longerSign);
    double b = round(upperEdgeReach() / longerSign);
    return new double[] {Math.min(a, b), Math.max(a, b)};
  }

  /** The middle of {@link #range}; NaN until a ball goes in. */
  public double centre() {
    return scoredReach.isEmpty() ? Double.NaN : round(centreReach() / longerSign);
  }

  /** Records where the ball fired at {@code fired} went, and works out the next setting. */
  public void record(double fired, BallMark mark) {
    double reach = longerSign * fired;
    balls++;
    if (mark.in) {
      scored++;
      scoredReach.add(reach);
      // A ball that scores past an edge means that edge's miss was noise
      if (reach >= shortestLongReach) { shortestLongReach = Double.POSITIVE_INFINITY; }
      if (reach <= longestShortReach) { longestShortReach = Double.NEGATIVE_INFINITY; }
    } else if (!scoredReach.isEmpty()
        && (mark.bounced || (mark.hasRange() && (reach > max(scoredReach) || reach < min(scoredReach))))) {
      boolean longSide = mark.bounced ? reach > mean(scoredReach) : reach > max(scoredReach);
      if (!mark.bounced && mark.longMiss != longSide) {
        contrary++;
      }
      if (longSide) {
        shortestLongReach = Math.min(shortestLongReach, reach);
      } else {
        longestShortReach = Math.max(longestShortReach, reach);
      }
    } else if (!mark.bounced && mark.hasRange()) {
      if (mark.longMiss) {
        shortestLongReach = Math.min(shortestLongReach, reach);
      } else {
        longestShortReach = Math.max(longestShortReach, reach);
      }
    }
    if (phase == Phase.CONFIRM && !mark.in && !mark.bounced && mark.hasRange()) {
      confirmMisses.add(new double[] {reach, mark.longMiss ? 1 : 0});
    }
    if (phase == Phase.CONFIRM || phase == Phase.DONE) {
      confirmFired++;
      confirmScored += mark.in ? 1 : 0;
      if (phase == Phase.CONFIRM && confirmFired >= confirmBalls && confirmMisses.size() * 2 > confirmFired) {
        // Most confirming balls missed on range: the search reopens, and the ins those misses contradict are
        // dropped as older (7 Nov, 085204: called found at 3 of 10 in, its three confirming balls all long, two ins
        // from an earlier session holding the range at -2.0 deg)
        for (double[] m : confirmMisses) {
          if (m[1] > 0) {
            scoredReach.removeIf(r -> r >= m[0]);
          } else {
            scoredReach.removeIf(r -> r <= m[0]);
          }
        }
        confirmMisses.clear();
        confirmFired = 0;
        confirmScored = 0;
        plan();
        return;
      }
      if (confirmFired >= confirmBalls) {
        phase = Phase.DONE;
      }
      nextReach = clamp(centreReach());
      return;
    }
    plan();
  }

  private void plan() {
    if (scoredReach.isEmpty()) {
      phase = Phase.FIND;
      boolean shortKnown = longestShortReach != Double.NEGATIVE_INFINITY;
      boolean longKnown = shortestLongReach != Double.POSITIVE_INFINITY;
      if (shortKnown && longKnown) {
        nextReach = (longestShortReach + shortestLongReach) / 2;
      } else if (shortKnown) {
        nextReach = longestShortReach + step;
      } else if (longKnown) {
        nextReach = shortestLongReach - step;
      }
      nextReach = clamp(nextReach);
      return;
    }
    double highest = max(scoredReach);
    double lowest = min(scoredReach);
    // Fired settings are rounded to the quantum, so a limit off it is reached within half of one (7 Nov, 082948: the
    // hood limit -3.93 deg, every ball fired at -3.9 went in and the long edge was never found, 38 balls)
    double atLimit = Math.max(1e-9, quantum / 2 + 1e-9);
    boolean longEdgeKnown = shortestLongReach - highest <= resolution || highest >= maxReach - atLimit;
    boolean shortEdgeKnown = lowest - longestShortReach <= resolution || lowest <= minReach + atLimit;
    if (!longEdgeKnown) {
      phase = Phase.LONG_EDGE;
      nextReach = shortestLongReach == Double.POSITIVE_INFINITY ? highest + step : (highest + shortestLongReach) / 2;
    } else if (!shortEdgeKnown) {
      phase = Phase.SHORT_EDGE;
      nextReach = longestShortReach == Double.NEGATIVE_INFINITY ? lowest - step : (longestShortReach + lowest) / 2;
    } else {
      phase = confirmBalls > 0 ? Phase.CONFIRM : Phase.DONE;
      nextReach = centreReach();
    }
    nextReach = clamp(nextReach);
  }

  private double upperEdgeReach() {
    double highest = max(scoredReach);
    return shortestLongReach == Double.POSITIVE_INFINITY ? highest
        : Math.max(highest, Math.min(maxReach, (highest + shortestLongReach) / 2));
  }

  private double lowerEdgeReach() {
    double lowest = min(scoredReach);
    return longestShortReach == Double.NEGATIVE_INFINITY ? lowest
        : Math.min(lowest, Math.max(minReach, (longestShortReach + lowest) / 2));
  }

  private double centreReach() {
    return (upperEdgeReach() + lowerEdgeReach()) / 2;
  }

  private double clamp(double reach) {
    return Math.max(minReach, Math.min(maxReach, reach));
  }

  private double round(double value) {
    return quantum > 0 ? Math.round(value / quantum) * quantum : value;
  }

  private static double max(List<Double> values) {
    double m = Double.NEGATIVE_INFINITY;
    for (double v : values) { m = Math.max(m, v); }
    return m;
  }

  private static double min(List<Double> values) {
    double m = Double.POSITIVE_INFINITY;
    for (double v : values) { m = Math.min(m, v); }
    return m;
  }

  private static double mean(List<Double> values) {
    double sum = 0;
    for (double v : values) { sum += v; }
    return sum / values.size();
  }
}
