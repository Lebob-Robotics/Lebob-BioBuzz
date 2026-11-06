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
  // A ball also counts as gone once the wheels fall this far below their highest reading since the feed began.
  // Below target alone missed balls fed with the wheels above it: 312 RPM had to go first, the feed ran on
  // 0.45 s and pushed a second ball out (5 Nov, 085458). The reading steps by about 60 RPM
  public static double DIP_FROM_PEAK_RPM = 150;
  // Each pulse after a pause with no ball out runs this much longer, so a ball sitting further back still goes
  public static double PULSE_GROWTH_S = 0.10;
  public static double PULSE_LIMIT_S = 1.0;

  // PAUSED: the pulse ran out with no dip seen yet; waits for the dip of a ball already on its way
  private enum State { WAITING, FEEDING, PAUSED, RECOVERING, DONE } // DONE: one-ball mode, until released or it ends

  private final double readySeconds;
  private final double stopUnreadyAfterS;
  private double pulseS;
  private final double pauseS;
  private final double minGapS;
  private double minFeedSpacingS = 0;
  private double lastFeedStartS = Double.NEGATIVE_INFINITY;
  private State state = State.WAITING;
  private double readySinceS = Double.NaN;
  private double feedingSinceS = Double.NaN;
  private double pausedSinceS = Double.NaN;
  private double ballLeftAtS = Double.NEGATIVE_INFINITY;
  private double peakRpm;
  private int emptyPulses;
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
    this(readySeconds, stopUnreadyAfterS, Double.POSITIVE_INFINITY, 0, 0);
  }

  /**
   * @param pulseS  the longest a feed runs before a ball is seen leaving; it then stops and waits up to
   *                {@code pauseS} for the dip of a ball already on its way, and otherwise feeds again once ready.
   *                The speed reading shows a ball 0.1-0.17 s after it left, by when the feeder had pushed the
   *                next one into the wheels (5 Nov video: second balls 0.13-0.4 s behind the first)
   * @param minGapS the shortest time from one ball leaving to the next feed (5 Nov, 084355: refed after 0.08 s)
   */
  public OneBallFeedGate(double readySeconds, double stopUnreadyAfterS, double pulseS, double pauseS,
      double minGapS) {
    this.readySeconds = readySeconds;
    this.stopUnreadyAfterS = stopUnreadyAfterS;
    this.pulseS = pulseS;
    this.pauseS = pauseS;
    this.minGapS = minGapS;
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
   * @param oneBall stop after the first ball until {@code held} is released, so each press shoots one. Once
   *                this goes false with {@code held} still true (RT pulled while RB is held), feeding resumes
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
      emptyPulses = 0;
      return false;
    }
    if (state == State.DONE && !oneBall) {
      state = State.WAITING;
    }
    switch (state) {
      case WAITING:
      case RECOVERING:
        if (steady && nowS - ballLeftAtS >= minGapS && nowS - lastFeedStartS >= minFeedSpacingS) {
          state = State.FEEDING;
          feedStarted = true;
          feedingSinceS = nowS;
          lastFeedStartS = nowS;
          peakRpm = rpm;
        }
        break;
      case FEEDING:
        peakRpm = Math.max(peakRpm, rpm);
        if (dipped(rpm, targetRpm)) {
          leave(nowS, oneBall);
        } else if (shotLost || (!atSpeed && nowS - feedingSinceS >= stopUnreadyAfterS)) {
          state = State.WAITING;
        } else if (nowS - feedingSinceS >= Math.min(PULSE_LIMIT_S, pulseS + emptyPulses * PULSE_GROWTH_S)) {
          state = State.PAUSED;
          pausedSinceS = nowS;
        }
        break;
      case PAUSED:
        if (dipped(rpm, targetRpm)) {
          leave(nowS, oneBall);
        } else if (nowS - pausedSinceS >= pauseS) {
          emptyPulses++;
          state = State.WAITING;
        }
        break;
      case DONE:
        break;
    }
    return state == State.FEEDING;
  }

  private boolean dipped(double rpm, double targetRpm) {
    return rpm < targetRpm - DIP_RPM || rpm < peakRpm - DIP_FROM_PEAK_RPM;
  }

  private void leave(double nowS, boolean oneBall) {
    state = oneBall ? State.DONE : State.RECOVERING;
    ballLeft = true;
    ballLeftAtS = nowS;
    readySinceS = Double.NaN;
    emptyPulses = 0;
  }

  /**
   * The shortest time from one feed starting to the next, from now on, whether or not a ball was seen leaving.
   * A feed whose ball the speed reading missed otherwise refeeds after the pause alone.
   */
  public void setMinFeedSpacingS(double minFeedSpacingS) {
    this.minFeedSpacingS = minFeedSpacingS;
  }

  /**
   * The longest a feed runs before pausing for its ball's dip, from now on. The dip shows a ball 0.1-0.17 s
   * after it left, and a second ball loaded tight behind the first went into the wheels with it.
   */
  public void setPulseS(double pulseS) {
    this.pulseS = pulseS;
  }

  /** For the log: WAITING, FEEDING, PAUSED, RECOVERING or DONE. */
  public String stateName() {
    return state.name();
  }

  /**
   * True once a pulse and its pause have passed with no ball out, until one leaves: the next ball is further back,
   * so the feeder should stop between pulses rather than hold backward and pull it back each time (6 Nov, 081243:
   * the last ball rocked at the bottom of the indexer for 10 s).
   */
  public boolean feederEmpty() {
    return emptyPulses > 0;
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
