package org.firstinspires.ftc.teamcode.utils;

/**
 * The backward shot's search: the RPM at one launch angle, stepping the hood steeper whenever that angle shows no
 * scoring RPM. A flat backward ball crosses the opening on the way past it, so it hits the bottom of the basket until
 * it is fast enough to clear the top, and then it is too fast to drop in: a short and a long within
 * {@code rpmResolution} of each other with no ball in. A steeper ball drops in from above and has a window.
 *
 * <p>Each angle's RPM search ({@link #rpm}) starts where the last one's short and long met.
 */
public final class BackwardShotSearch {
  private final double steepenDeg;
  private final double minLaunchDeg;
  private final double rpmStep;
  private final double rpmResolution;
  private final double minRpm;
  private final double maxRpm;
  private final int confirmBalls;

  private double launchDeg;
  private ShotSearch rpmSearch;
  private int steepened;

  /**
   * @param startLaunchDeg the table's launch angle
   * @param startRpm the table's RPM
   * @param steepenDeg how far the launch angle drops (toward straight up) when an angle has no window
   * @param minLaunchDeg the steepest launch angle tried
   */
  public BackwardShotSearch(double startLaunchDeg, double startRpm, double steepenDeg, double minLaunchDeg,
      double rpmStep, double rpmResolution, double minRpm, double maxRpm, int confirmBalls) {
    this.steepenDeg = steepenDeg;
    this.minLaunchDeg = minLaunchDeg;
    this.rpmStep = rpmStep;
    this.rpmResolution = rpmResolution;
    this.minRpm = minRpm;
    this.maxRpm = maxRpm;
    this.confirmBalls = confirmBalls;
    launchDeg = startLaunchDeg;
    rpmSearch = newRpmSearch(startRpm);
  }

  /** The RPM search at the current launch angle. */
  public ShotSearch rpm() { return rpmSearch; }

  /** The launch angle to fire at. */
  public double launchDeg() { return launchDeg; }

  /** How many times the hood has been stepped steeper. */
  public int steepened() { return steepened; }

  /** Records a ball fired at {@code rpm} at the current launch angle. */
  public void record(double rpm, BallMark mark) {
    rpmSearch.record(rpm, mark);
    double furthestShort = rpmSearch.furthestShort();
    double nearestLong = rpmSearch.nearestLong();
    boolean noWindow = rpmSearch.scored() == 0 && !Double.isNaN(furthestShort) && !Double.isNaN(nearestLong)
        && nearestLong - furthestShort <= rpmResolution + 1e-9;
    if (noWindow && launchDeg > minLaunchDeg + 1e-9) {
      launchDeg = Math.max(minLaunchDeg, Math.round((launchDeg - steepenDeg) * 10) / 10.0);
      steepened++;
      rpmSearch = newRpmSearch((furthestShort + nearestLong) / 2);
    }
  }

  private ShotSearch newRpmSearch(double start) {
    return new ShotSearch(start, 1, rpmStep, rpmResolution, 10, minRpm, maxRpm, confirmBalls);
  }
}
