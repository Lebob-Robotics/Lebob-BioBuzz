package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Checks that the REV moves with the hood motor: over each stretch of {@link #WINDOW_MOTOR_TICKS} of motor
 * travel the REV must move at least {@link #SHARE} as far, the same way. Both are on the pinion, 1:1
 * through the belt, so a REV that falls behind this far is unplugged, loose on its shaft, or the belt is
 * skipping wholesale. Either way the REV no longer says where the hood is, and driving on it ran the hood
 * past its top (085313).
 *
 * <p>Sized on the logs of 5 Nov: 60 ticks clears reversal backlash (16) and cross-hub read skew; only the
 * real failures (085313, and 081558 where the REV stood still while the motor ran 260 ticks) fall short.
 */
final class HoodFollowCheck {
  static final int WINDOW_MOTOR_TICKS = 60;
  static final double SHARE = 0.25;

  private boolean anchored;
  private int anchorRev;
  private int anchorMotor;

  void reset() {
    anchored = false;
  }

  /** Counts on the same pinion, each against any fixed zero. True once the REV failed to follow a window. */
  boolean update(int revTicks, int motorTicks) {
    if (!anchored) {
      anchored = true;
      anchorRev = revTicks;
      anchorMotor = motorTicks;
      return false;
    }
    int motorMoved = motorTicks - anchorMotor;
    if (Math.abs(motorMoved) < WINDOW_MOTOR_TICKS) {
      return false;
    }
    double revMoved = (revTicks - anchorRev) * HoodMotorTargeting.MOTOR_PER_REV;
    anchorRev = revTicks;
    anchorMotor = motorTicks;
    return revMoved * Math.signum(motorMoved) < SHARE * Math.abs(motorMoved);
  }
}
