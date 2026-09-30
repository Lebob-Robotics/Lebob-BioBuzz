package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Stall protection for the indexer while it feeds the shooter. Pure logic with
 * no hardware, so it can be unit tested.
 *
 * <p>
 * Measured on the prototype on 30 Sep: jammed on two balls, the indexer drew
 * 8.4 to 11.2 A at 0 ticks/s, while heavily loaded but still moving it turned
 * at 460 ticks/s or more. So high current at near-zero speed for
 * {@link #STALL_TIME_S} means jammed. Each jam backs the indexer off for
 * {@link #UNJAM_TIME_S} and then feeding resumes. After
 * {@link #MAX_UNJAM_ATTEMPTS} back-offs in one feed, the next jam leaves the
 * indexer off until the next feed starts, so a hard jam does not cook the
 * motor.
 */
public class IndexerStallGuard {
  public static final double STALL_CURRENT_AMPS = 7.0;
  /** Ticks per second. */
  public static final double STALL_VELOCITY = 150;
  public static final double STALL_TIME_S = 0.3;
  public static final double UNJAM_TIME_S = 0.2;
  public static final int MAX_UNJAM_ATTEMPTS = 3;

  public enum Phase {
    FEEDING, UNJAMMING, JAMMED
  }

  private final double feedPower;
  private final double unjamPower;

  private Phase phase = Phase.FEEDING;
  private int unjamAttempts;
  private double phaseStartS;
  private double stalledSinceS = Double.NaN;

  /**
   * @param feedPower  indexer power while feeding
   * @param unjamPower indexer power while backing off a jam (negative)
   */
  public IndexerStallGuard(double feedPower, double unjamPower) {
    this.feedPower = feedPower;
    this.unjamPower = unjamPower;
  }

  /** Starts a new feed, clearing any jam and the attempt count. */
  public void start(double nowS) {
    unjamAttempts = 0;
    enter(Phase.FEEDING, nowS);
  }

  /**
   * @param nowS        time in seconds
   * @param currentAmps indexer current draw
   * @param velocity    indexer velocity in ticks/second
   * @return the power to apply to the indexer
   */
  public double update(double nowS, double currentAmps, double velocity) {
    switch (phase) {
      case FEEDING:
        boolean stalled = currentAmps > STALL_CURRENT_AMPS && Math.abs(velocity) < STALL_VELOCITY;
        if (!stalled) {
          stalledSinceS = Double.NaN;
        } else if (Double.isNaN(stalledSinceS)) {
          stalledSinceS = nowS;
        } else if (nowS - stalledSinceS >= STALL_TIME_S) {
          if (unjamAttempts < MAX_UNJAM_ATTEMPTS) {
            unjamAttempts++;
            enter(Phase.UNJAMMING, nowS);
          } else {
            enter(Phase.JAMMED, nowS);
          }
        }
        break;
      case UNJAMMING:
        if (nowS - phaseStartS >= UNJAM_TIME_S) {
          enter(Phase.FEEDING, nowS);
        }
        break;
      case JAMMED:
        break;
    }
    return getPower();
  }

  private void enter(Phase newPhase, double nowS) {
    phase = newPhase;
    phaseStartS = nowS;
    stalledSinceS = Double.NaN;
  }

  /** The power the indexer should be at in the current phase. */
  public double getPower() {
    switch (phase) {
      case FEEDING:
        return feedPower;
      case UNJAMMING:
        return unjamPower;
      default:
        return 0.0;
    }
  }

  public Phase getPhase() {
    return phase;
  }

  public int getUnjamAttempts() {
    return unjamAttempts;
  }
}
