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

  private enum State { WAITING, FEEDING, RECOVERING, DONE } // DONE: one-ball mode, until released

  private final double readySeconds;
  private final double stopUnreadyAfterS;
  private State state = State.WAITING;
  private double readySinceS = Double.NaN;
  private double feedingSinceS = Double.NaN;
  private boolean ballLeft;
  private boolean feedStarted;

  /** Waits {@link #READY_SECONDS}. */
  public OneBallFeedGate() {
    this(Double.NaN);
  }

  /** @param readySeconds how long the wheels must hold speed before a feed, or NaN for {@link #READY_SECONDS} */
  public OneBallFeedGate(double readySeconds) {
    this(readySeconds, Double.POSITIVE_INFINITY);
  }

  /**
   * @param stopUnreadyAfterS a feed that has gone this long without a ball leaving stops once no longer
   *                          ready, and waits for ready again. Otherwise an empty feeder kept running, and
   *                          the next ball loaded went straight out whatever the aim (5 Nov, 094738: 11 s)
   */
  public OneBallFeedGate(double readySeconds, double stopUnreadyAfterS) {
    this.readySeconds = readySeconds;
    this.stopUnreadyAfterS = stopUnreadyAfterS;
  }

  /**
   * @param held       the driver wants to shoot
   * @param atSpeed    both wheels inside the shot's RPM tolerance this loop
   * @param rpm        mean wheel speed
   * @param targetRpm  the speed the wheels are held at
   * @return whether to run the feeder and indexer this loop
   */
  public boolean update(double nowS, boolean held, boolean atSpeed, double rpm, double targetRpm) {
    return update(nowS, held, atSpeed, rpm, targetRpm, false);
  }

  /**
   * As above.
   *
   * @param oneBall stop after the first ball until {@code held} is released, so each press shoots one
   */
  public boolean update(double nowS, boolean held, boolean atSpeed, double rpm, double targetRpm,
      boolean oneBall) {
    return update(nowS, held, atSpeed, rpm, targetRpm, oneBall, false);
  }

  /**
   * As above.
   *
   * @param shotLost the shot went bad after the feed started (aim, hood or a valid shot lost, not the wheel
   *                 speed, whose reading steps): a feed with no ball out yet stops and waits for ready again
   */
  public boolean update(double nowS, boolean held, boolean atSpeed, double rpm, double targetRpm,
      boolean oneBall, boolean shotLost) {
    ballLeft = false;
    feedStarted = false;
    if (!atSpeed) {
      readySinceS = Double.NaN;
    } else if (Double.isNaN(readySinceS)) {
      readySinceS = nowS;
    }
    double waitS = Double.isNaN(readySeconds) ? READY_SECONDS : readySeconds;
    boolean steady = !Double.isNaN(readySinceS) && nowS - readySinceS >= waitS;

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
          feedingSinceS = nowS;
        }
        break;
      case FEEDING:
        if (rpm < targetRpm - DIP_RPM) {
          state = oneBall ? State.DONE : State.RECOVERING;
          ballLeft = true;
          readySinceS = Double.NaN;
        } else if (shotLost || (!atSpeed && nowS - feedingSinceS >= stopUnreadyAfterS)) {
          state = State.WAITING;
        }
        break;
      case DONE:
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
