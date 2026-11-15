package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * SOTM Tuning's search at one moving spot (one steady pass, fired at the same point every time): the range, as a
 * trim on the moving shot's launch angle or RPM, found by a {@link ShotSearch} exactly as Shot Tuning finds a
 * standing spot's, and the aim, as a heading trim moved by {@link AimTrimSearch} from the left and right marks.
 * Each ball is recorded with the range trim it was fired with; a changed mark replays every ball.
 */
public final class SotmSpotSearch {
  public final boolean rpm; // The range is the RPM trim, else the launch angle trim
  public final double minTrim;
  public final double maxTrim;
  public final double startHeadingTrimDeg;
  private final double rangeStep;
  private final double rangeResolution;
  private final int confirmBalls;
  private final double headingStepDeg;
  private final double headingMinStepDeg;
  private final List<Double> fired = new ArrayList<>();
  private final List<Double> firedHeading = new ArrayList<>(); // The aim trim each ball was fired with
  private final List<BallMark> marks = new ArrayList<>();
  private ShotSearch range;
  private AimTrimSearch heading;

  /**
   * @param rpm true to search the RPM trim (the shot's hood near a stop, or the backward shot), else the launch angle
   * @param minTrim lowest trim allowed, so the shot stays inside the hood's travel or the wheels' speed
   * @param maxTrim highest trim allowed
   * @param startHeadingTrimDeg the aim trim the first ball is fired with, the standing one
   */
  public SotmSpotSearch(boolean rpm, double minTrim, double maxTrim, double startHeadingTrimDeg,
      double rangeStep, double rangeResolution, int confirmBalls, double headingStepDeg, double headingMinStepDeg) {
    this.rpm = rpm;
    this.minTrim = Math.min(minTrim, 0);
    this.maxTrim = Math.max(maxTrim, 0);
    this.startHeadingTrimDeg = startHeadingTrimDeg;
    this.rangeStep = rangeStep;
    this.rangeResolution = rangeResolution;
    this.confirmBalls = confirmBalls;
    this.headingStepDeg = headingStepDeg;
    this.headingMinStepDeg = headingMinStepDeg;
    reset();
  }

  /** With Shot Tuning's search settings. */
  public static SotmSpotSearch withTuningSettings(boolean rpm, double minTrim, double maxTrim,
      double startHeadingTrimDeg) {
    return new SotmSpotSearch(rpm, minTrim, maxTrim, startHeadingTrimDeg,
        rpm ? ShotTuningSettings.SEARCH_RPM_STEP : ShotTuningSettings.SEARCH_HOOD_STEP_DEG,
        rpm ? ShotTuningSettings.SEARCH_RPM_RESOLUTION : ShotTuningSettings.SEARCH_HOOD_RESOLUTION_DEG,
        ShotTuningSettings.CONFIRM_BALLS, ShotTuningSettings.TRIM_STEP_DEG, ShotTuningSettings.TRIM_MIN_STEP_DEG);
  }

  private void reset() {
    // More launch angle is a steeper, shorter shot; more RPM a longer one
    range = new ShotSearch(0, rpm ? 1 : -1, rangeStep, rangeResolution, rpm ? 10 : 0.1, minTrim, maxTrim,
        confirmBalls);
    heading = new AimTrimSearch(headingStepDeg, headingMinStepDeg);
    heading.set(startHeadingTrimDeg);
  }

  /** The range trim to fire next: launch angle degrees added to the shot's, or RPM. */
  public double rangeTrim() { return range.next(); }

  /** The aim trim to fire next, degrees anticlockwise. */
  public double headingTrimDeg() { return heading.trimDeg(); }

  /** A ball fired with {@code rangeTrimFired}, marked for the first time. */
  public void record(double rangeTrimFired, BallMark mark) {
    record(rangeTrimFired, heading.trimDeg(), mark);
  }

  /** As above, with the aim trim it was fired with: a ball read back from the CSV. NaN if not known. */
  public void record(double rangeTrimFired, double headingTrimFiredDeg, BallMark mark) {
    fired.add(rangeTrimFired);
    firedHeading.add(headingTrimFiredDeg);
    marks.add(mark);
    range.record(rangeTrimFired, mark);
    heading.record(mark, headingTrimFiredDeg);
  }

  /** The last ball's mark was changed: every ball is replayed with it. */
  public void remark(BallMark mark) {
    if (marks.isEmpty()) {
      return;
    }
    marks.set(marks.size() - 1, mark);
    reset();
    for (int i = 0; i < fired.size(); i++) {
      range.record(fired.get(i), marks.get(i));
      heading.record(marks.get(i), firedHeading.get(i));
    }
  }

  public boolean done() { return range.phase() == ShotSearch.Phase.DONE; }
  public ShotSearch.Phase phase() { return range.phase(); }
  public int balls() { return range.balls(); }
  public int scored() { return range.scored(); }
  public double[] range() { return range.range(); }
  public double centre() { return range.centre(); }
  public double headingStepDeg() { return heading.stepDeg(); }
}
