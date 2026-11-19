package org.firstinspires.ftc.teamcode.robot;

/**
 * A shoot button (RT or RB) let go mid-feed stays held until that feed ends, at most
 * {@link DriverConstants#SHOT_RELEASE_HOLD_S}: stopped at once, a ball halfway through the feeder was left there or
 * went out unseen, and the ball count went wrong. Let go between feeds, it stops at once.
 */
final class ShotHoldover {
  private boolean wasHeld = false;
  private boolean active = false;
  private double releasedAtS = Double.NaN;

  /**
   * @param held    the button is down
   * @param feeding a feed is running
   * @return the button as the robot should act on it
   */
  boolean update(double nowS, boolean held, boolean feeding) {
    if (held) {
      wasHeld = true;
      active = false;
      return true;
    }
    if (wasHeld) {
      active = feeding;
      releasedAtS = nowS;
    }
    wasHeld = false;
    if (active && (!feeding || nowS - releasedAtS >= DriverConstants.SHOT_RELEASE_HOLD_S)) {
      active = false;
    }
    return active;
  }
}
