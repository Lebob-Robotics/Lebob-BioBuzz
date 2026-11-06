package org.firstinspires.ftc.teamcode.subsystems;

/**
 * What the hood does about a loose belt, so one bad move does not leave it dead until the next INIT.
 *
 * <p>A slip (the motor ran but the REV did not follow) is retried at a lower speed: the belt skips under
 * the torque of a full-power start. A guard trip, which the guard latches after repeated slips, is
 * answered by driving down onto the lower stop again, where the belt engages, and trying afresh. After
 * {@code maxRehomes} of those in a row without the hood reaching a target, the fault stays latched, so
 * an unplugged or reversed encoder still stops the hood.
 */
final class HoodRecovery {
  private final int maxRehomes;
  private int rehomes;
  private int slipsSinceArrival;

  HoodRecovery(int maxRehomes) {
    this.maxRehomes = maxRehomes;
  }

  /** The motor ran and the REV did not follow. */
  void onSlip() {
    slipsSinceArrival++;
  }

  /** True if the hood should drive down and re-zero rather than stay faulted. */
  boolean onTrip() {
    if (rehomes >= maxRehomes) {
      return false;
    }
    rehomes++;
    slipsSinceArrival = 0;
    return true;
  }

  /** The hood reached its target on the REV, so the belt is holding: start counting afresh. */
  void onArrived() {
    rehomes = 0;
    slipsSinceArrival = 0;
  }

  void reset() {
    onArrived();
  }

  int rehomes() {
    return rehomes;
  }

  int slipsSinceArrival() {
    return slipsSinceArrival;
  }

  /** Full speed until something goes wrong, then a share less for each slip or re-home, never below floor. */
  double speed(double maxSpeed, double factorPerRetry, double floor) {
    double speed = maxSpeed * Math.pow(factorPerRetry, slipsSinceArrival + rehomes);
    return Math.max(Math.min(floor, maxSpeed), Math.min(maxSpeed, speed));
  }
}
