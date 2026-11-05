package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Turns a hood target on the REV Through Bore into a RUN_TO_POSITION target on the hood motor's
 * own encoder, so the hub runs the fast position loop and the REV only corrects where it aims.
 *
 * <p>The offset is the motor reading minus the REV reading in motor ticks. The belt has about
 * 16-20 motor ticks of lost motion, so the offset differs by that much depending on which way the
 * belt was last driven. One offset is kept per direction and the next move uses the one for its
 * own direction, so a reversal goes straight to the right place instead of stopping short and
 * correcting a loop later.
 *
 * <p>Offsets are learned only once the motor has finished its move (at rest on its own target):
 * mid-move the two hubs' readings are skewed, and a motor stalled short of its target, or one that
 * has not yet started a reversal, still has the belt loaded the old way. Within the deadband they
 * are left alone, so a settled hood does not hunt by one tick.
 */
final class HoodMotorTargeting {
  /** Motor encoder ticks per REV tick: 384.5 vs 8192 counts per pinion turn, 1:1. */
  static final double MOTOR_PER_REV =
      HoodEncoderGuard.MOTOR_COUNTS_PER_TURN / HoodEncoderGuard.REV_COUNTS_PER_TURN;

  private int offsetUp;
  private int offsetDown;
  private int direction; // +1 up, -1 down, 0 none yet: the way the belt was last driven

  void reset() {
    offsetUp = 0;
    offsetDown = 0;
    direction = 0;
  }

  /** The offset the current move uses. */
  int offset() {
    return direction < 0 ? offsetDown : offsetUp;
  }

  int offsetUp() {
    return offsetUp;
  }

  int offsetDown() {
    return offsetDown;
  }

  /**
   * Re-anchors the last-driven direction's offset when the motor has finished its move and the
   * hood is still off target.
   */
  void observe(int revTicks, int motorTicks, boolean moveDone, int revTargetTicks, int deadbandTicks) {
    if (!moveDone || Math.abs(revTargetTicks - revTicks) < deadbandTicks) {
      return;
    }
    int learned = motorTicks - (int) Math.round(revTicks * MOTOR_PER_REV);
    if (direction < 0) {
      offsetDown = learned;
    } else {
      offsetUp = learned;
    }
  }

  /**
   * Motor-encoder target, relative to the motor's lower-stop zero, using the offset for the
   * direction the hood must move. Inside the deadband the direction is left as it was.
   */
  int motorTargetFor(int revTargetTicks, int revTicks, int deadbandTicks) {
    return motorTargetFor(revTargetTicks, revTicks, deadbandTicks, false);
  }

  /**
   * As above. While the motor is still moving the direction is held: the REV error changes sign during
   * an overshoot, and switching offsets mid-move sent the motor 1.4 degrees back the other way, which
   * set up a back-and-forth that ended up to 1.9 degrees off (Hood Tune, 5 Nov).
   */
  int motorTargetFor(int revTargetTicks, int revTicks, int deadbandTicks, boolean motorMoving) {
    int error = revTargetTicks - revTicks;
    if (Math.abs(error) >= deadbandTicks && (!motorMoving || direction == 0)) {
      direction = error > 0 ? 1 : -1;
    }
    return (int) Math.round(revTargetTicks * MOTOR_PER_REV) + offset();
  }
}
