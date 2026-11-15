package org.firstinspires.ftc.teamcode.tuning;

/**
 * Whether a feed carried a ball: it did once the flywheels dip {@link #DIP_RPM} below their speed as the feed
 * started, within {@link #WINDOW_S}. The tuning OpModes start it with each feed and write the ball's row on BALL.
 */
public final class BallDipDetector {
  // The one-ball gate pulses the feeder again ~0.8 s after a pulse that delivered nothing; those dipped under 150 rpm
  // (6 Nov 082517)
  private static final double DIP_RPM = 200;
  private static final double WINDOW_S = 0.6;

  /** What the feed has shown so far. */
  public enum Result { WAITING, BALL, EMPTY }

  private double startS;
  private double startRpm;
  private double minRpm;

  /** A feed started, or a retry pulse replaced the last: a ball dips the wheels from this speed. */
  public void start(double nowS, double rpm) {
    startS = nowS;
    startRpm = rpm;
    minRpm = rpm;
  }

  /** This loop's flywheel speed, each loop from {@link #start} until BALL or EMPTY. */
  public Result update(double nowS, double rpm) {
    minRpm = Math.min(minRpm, rpm);
    if (dipRpm() >= DIP_RPM) {
      return Result.BALL;
    }
    return nowS - startS > WINDOW_S ? Result.EMPTY : Result.WAITING;
  }

  /** How far the wheels have dipped since the feed started. */
  public double dipRpm() {
    return startRpm - minRpm;
  }

  public double sinceStartS(double nowS) {
    return nowS - startS;
  }
}
