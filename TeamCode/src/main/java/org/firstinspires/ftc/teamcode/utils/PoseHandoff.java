package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * The autonomous's last pose and alliance, for the TeleOp that follows it. Every OpMode resets the Pinpoint, and
 * TeleOp used to start at the "placed square" heading: after the Scoring Auto ended at -129 degrees, TeleOp
 * started at +90 and drove 140 degrees rotated until its first two-tag fix, 4.7 s in (13 Nov 224556 -> 225324).
 * The first fix took 2.9 s typically, and 26-118 s at worst. In a match nobody touches the robot in between.
 *
 * <p>Held in a static, so it lives as long as the Robot Controller app does. The pose is used once, and only while
 * fresh; the alliance carries on until another auto runs.
 */
public final class PoseHandoff {
  /** The pose is only used this soon after the auto stopped: a match's transition is about 8 s. */
  public static double MAX_AGE_S = 60;

  private static Pose pose;
  private static long savedNanos;
  private static Alliance alliance;

  private PoseHandoff() {}

  /** The auto's pose as it stops (null if it never knew where it was), and its alliance. */
  public static synchronized void save(Pose lastPose, Alliance lastAlliance, long nowNanos) {
    pose = lastPose;
    savedNanos = nowNanos;
    alliance = lastAlliance;
  }

  /** The last auto's alliance, or null if none ran since the app started. */
  public static synchronized Alliance alliance() {
    return alliance;
  }

  /** The auto's pose if it is under MAX_AGE_S old, else null. Doesn't use it up. */
  public static synchronized Pose peek(long nowNanos) {
    return pose != null && (nowNanos - savedNanos) * 1e-9 <= MAX_AGE_S ? pose : null;
  }

  /** As {@link #peek}, then forgets the pose, so a later practice TeleOp doesn't start from it. */
  public static synchronized Pose take(long nowNanos) {
    Pose p = peek(nowNanos);
    pose = null;
    return p;
  }

  /** Forgets everything (tests). */
  public static synchronized void clear() {
    pose = null;
    alliance = null;
  }
}
