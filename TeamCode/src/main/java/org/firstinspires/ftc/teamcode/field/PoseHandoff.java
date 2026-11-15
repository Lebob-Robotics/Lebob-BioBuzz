package org.firstinspires.ftc.teamcode.field;


/**
 * The autonomous's last pose and alliance, for the TeleOp that follows it. Every OpMode resets the Pinpoint, and
 * TeleOp used to start at the "placed square" heading, driving rotated until its first two-tag fix, which can take
 * seconds. In a match nobody touches the robot in between.
 *
 * <p>Held in a static, so it lives as long as the Robot Controller app does. The pose is used once, and only while
 * fresh; the alliance carries on until another auto runs.
 */
public final class PoseHandoff {
  /** The pose is only used this soon after the auto stopped: a match's transition is about 8 s. */
  public static double MAX_AGE_S = 60;

  private static FieldPose pose;
  private static long savedNanos;
  private static Alliance alliance;

  private PoseHandoff() {}

  /** The auto's pose as it stops (null if it never knew where it was), and its alliance. */
  public static synchronized void save(FieldPose lastPose, Alliance lastAlliance, long nowNanos) {
    pose = lastPose;
    savedNanos = nowNanos;
    alliance = lastAlliance;
  }

  /** The last auto's alliance, or null if none ran since the app started. */
  public static synchronized Alliance alliance() {
    return alliance;
  }

  /** The auto's pose if it is under MAX_AGE_S old, else null. Doesn't use it up. */
  public static synchronized FieldPose peek(long nowNanos) {
    return pose != null && (nowNanos - savedNanos) * 1e-9 <= MAX_AGE_S ? pose : null;
  }

  /** As {@link #peek}, then forgets the pose, so a later practice TeleOp doesn't start from it. */
  public static synchronized FieldPose take(long nowNanos) {
    FieldPose p = peek(nowNanos);
    pose = null;
    return p;
  }

  /** Forgets everything (tests). */
  public static synchronized void clear() {
    pose = null;
    alliance = null;
  }
}
