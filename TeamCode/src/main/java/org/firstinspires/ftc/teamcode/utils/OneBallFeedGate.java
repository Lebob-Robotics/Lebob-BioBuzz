package org.firstinspires.ftc.teamcode.utils;

/**
 * Feeds one ball at a time, each at full flywheel speed.
 *
 * <p>Waits until the wheels have held speed for {@link #READY_SECONDS}, then feeds without
 * stopping until a ball goes through. The ball shows up as the wheels dropping more than
 * {@link #DIP_RPM}. The feed then pauses until the wheels are back up to speed before the next ball.
 *
 * <p>The hub's speed reading moves in steps of about 60 RPM. Gating every loop on a ±50 RPM band
 * started and stopped the feed every few loops, so each ball got a different push. The wheels
 * also lost about 430 RPM per ball, so a ball fed straight after another left slower.
 */
public final class OneBallFeedGate {
  public static double READY_SECONDS = 0.10;
  public static double DIP_RPM = 200;

  private enum State { WAITING, FEEDING, RECOVERING }

  private State state = State.WAITING;
  private double readySinceS = Double.NaN;
  private boolean ballLeft;
  private boolean feedStarted;

  /**
   * @param held       the driver wants to shoot
   * @param atSpeed    both wheels inside the shot's RPM tolerance this loop
   * @param rpm        mean wheel speed
   * @param targetRpm  the speed the wheels are held at
   * @return whether to run the feeder and indexer this loop
   */
  public boolean update(double nowS, boolean held, boolean atSpeed, double rpm, double targetRpm) {
    ballLeft = false;
    feedStarted = false;
    if (!atSpeed) {
      readySinceS = Double.NaN;
    } else if (Double.isNaN(readySinceS)) {
      readySinceS = nowS;
    }
    boolean steady = !Double.isNaN(readySinceS) && nowS - readySinceS >= READY_SECONDS;

    if (!held) {
      state = State.WAITING;
      return false;
    }
    switch (state) {
      case WAITING:
      case RECOVERING:
        if (steady) {
          state = State.FEEDING;
          feedStarted = true;
        }
        break;
      case FEEDING:
        if (rpm < targetRpm - DIP_RPM) {
          state = State.RECOVERING;
          ballLeft = true;
          readySinceS = Double.NaN;
        }
        break;
    }
    return state == State.FEEDING;
  }

  /** True on the loop a ball was seen leaving (the wheels dipped). */
  public boolean ballLeft() {
    return ballLeft;
  }

  /** True on the loop feeding started at full speed: the moment to record the shot's speed and angle. */
  public boolean feedStarted() {
    return feedStarted;
  }
}
