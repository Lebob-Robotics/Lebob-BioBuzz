package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Stall protection for the two shooter motors. Pure logic with no hardware, so
 * it can be unit tested.
 *
 * <p>
 * Measured on Mark 3 on 25 Oct, when the shooter pulleys were on backwards and
 * two motors burnt out: at full power both motors drew 6.4 to 7 A while turning
 * only 480 to 700 ticks/s, for about 3 minutes across several runs. Once one
 * motor died, the other drew 7 to 9.6 A at 0 ticks/s. Running freely the same
 * motors drew 0.3 to 0.5 A at 2300 to 2460 ticks/s. So the overload was slow
 * but still moving, and a zero-speed check would have missed it: a motor
 * counts as stalled above {@link #STALL_CURRENT_AMPS} while below
 * {@link #STALL_VELOCITY}.
 *
 * <p>
 * Either motor stalled for {@link #STALL_TIME_S} trips the guard, which keeps
 * the shooter off until {@link #rearm()}. After {@link #MAX_TRIPS} trips the
 * guard stays tripped until the OpMode restarts: a fault that keeps coming back
 * is mechanical and needs someone to look at the robot.
 */
public class ShooterStallGuard {
  public static final double STALL_CURRENT_AMPS = 4.0;
  /** Ticks per second, about half the free speed. */
  public static final double STALL_VELOCITY = 1200;
  /**
   * Long enough for a normal spin-up from rest to pass {@link #STALL_VELOCITY}.
   * Not yet measured with the belts on: check the Shooter/StallGuard log.
   */
  public static final double STALL_TIME_S = 1.5;
  public static final int MAX_TRIPS = 3;

  private boolean tripped;
  private int trips;
  private double leftStalledSinceS = Double.NaN;
  private double rightStalledSinceS = Double.NaN;

  /**
   * @param nowS       time in seconds
   * @param powered    true if the shooter is commanded to run
   * @param leftAmps   left motor current draw
   * @param leftVel    left motor velocity in ticks/second
   * @param rightAmps  right motor current draw
   * @param rightVel   right motor velocity in ticks/second
   * @return true if the shooter must be off
   */
  public boolean update(double nowS, boolean powered,
      double leftAmps, double leftVel, double rightAmps, double rightVel) {
    if (tripped) {
      return true;
    }
    if (!powered) {
      clearTimers();
      return false;
    }
    leftStalledSinceS = stalledSince(leftStalledSinceS, nowS, leftAmps, leftVel);
    rightStalledSinceS = stalledSince(rightStalledSinceS, nowS, rightAmps, rightVel);
    if (nowS - leftStalledSinceS >= STALL_TIME_S || nowS - rightStalledSinceS >= STALL_TIME_S) {
      tripped = true;
      trips++;
      clearTimers();
    }
    return tripped;
  }

  private static double stalledSince(double sinceS, double nowS, double amps, double velocity) {
    if (amps <= STALL_CURRENT_AMPS || Math.abs(velocity) >= STALL_VELOCITY) {
      return Double.NaN;
    }
    return Double.isNaN(sinceS) ? nowS : sinceS;
  }

  private void clearTimers() {
    leftStalledSinceS = Double.NaN;
    rightStalledSinceS = Double.NaN;
  }

  /** Clears a trip, unless the guard has tripped {@link #MAX_TRIPS} times. */
  public void rearm() {
    if (!isLockedOut()) {
      tripped = false;
    }
  }

  public boolean isTripped() {
    return tripped;
  }

  /** True once tripped {@link #MAX_TRIPS} times; only an OpMode restart clears it. */
  public boolean isLockedOut() {
    return trips >= MAX_TRIPS;
  }

  public int getTrips() {
    return trips;
  }
}
