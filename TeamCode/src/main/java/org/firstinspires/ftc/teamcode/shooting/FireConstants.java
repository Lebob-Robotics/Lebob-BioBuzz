package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/**
 * When an aimed shot fires and how its feed runs, one ball at a time (VolleyConstants for bursts). The tolerances
 * match tools/shots/inputs.json. Editable live from Panels.
 */
@Configurable
public final class FireConstants {
  private FireConstants() {}

  // Per flywheel. The hub's speed reading steps by about 61 RPM, and the wheels settle a step or so above
  // target (3605 read as 3628-3689, 4 Nov), so 75 held the shot back about 3 s. Two steps
  public static double RPM_TOLERANCE = 125;
  // The two wheels' mean, which the aimed shot also needs. The shot table allows 50 RPM, and 50 RPM is about
  // 0.1 m of range. Both misses of 5 Nov (080106) left 56 and 80-98 RPM fast, one long; balls 63 RPM slow went in,
  // and the wheels lag a target that moves with the robot's speed, so slow gets more room
  public static double RPM_MEAN_ABOVE_TARGET = 50;
  public static double RPM_MEAN_BELOW_TARGET = 100;
  // The speed this far ahead, from its trend, must not be more than this over: wheels climbing back after a ball
  // were fed at +17 and the ball left at +130, long (5 Nov, 080706). Fast side only, and looser than the speed
  // now: balls predicted +61 and +85 went in, and wheels slowing toward the target settle rather than undershoot.
  // The mean steps 30 RPM, so the slope needs 0.2 s
  public static double RPM_PREDICTED_ABOVE_TARGET = 100;
  public static double RPM_LOOKAHEAD_S = 0.08;
  public static double RPM_TREND_WINDOW_S = 0.2;
  public static boolean PRESPIN_ON = true; // Flywheels track the aimed RPM whenever the Cell is known
  public static double PRESPIN_HOLD_S = 1.5; // Keeps that speed this long after the shot goes invalid
  public static double HOOD_TOLERANCE_DEG = 1.0;
  // Otherwise it grows large at short range. The geometric clearance leaves nothing for the other errors: every
  // moving ball that scored left within 1.1 degrees, the misses 7.7 and 13.6 degrees off, all going the way the
  // robot moved (5 Nov, 083109-092330). 4 keeps 92% of moving READY time (soak 081210, median 1.6)
  public static double MAX_HEADING_TOLERANCE_DEG = 4;
  // No shot while moving at least MOVING_SHOT_SPEED_MPS this close to the lip, nor while the hood's target swings
  // faster than MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC. Strafing past at 0.7-0.8 m/s, balls fed READY at 0.62 and 0.48 m
  // both missed: during the 0.3 s feed the heading error went 7.7 -> 20.8 degrees and the hood target moved
  // 6 degrees; the standing shot from 1.5 m went in (5 Nov, 083109). Costs about a quarter of moving READY time
  public static double MOVING_SHOT_SPEED_MPS = 0.3;
  public static double MIN_MOVING_SHOT_LIP_M = 0.7;
  public static double MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = 12;
  // How long an aimed shot must stay ready before the feed starts. Every check above already has to pass,
  // and waiting two or three loops for it to repeat cost more shots than it saved while moving (5 Nov, 084358)
  public static double AIMED_READY_S = 0.0;
  // A feed with no ball out after this long stops once the shot is no longer ready. Past FEED_DELAY_MAX_S, so a
  // ball already on its way isn't cut off
  public static double AIMED_FEED_STOP_UNREADY_S = 0.4;
  // The longest an aimed feed runs before a ball is seen leaving. The webcam saw first balls leave 0.15-0.28 s
  // after the feed began and second balls 0.13-0.4 s behind them, while the wheel speed showed a ball 0.1-0.17 s
  // late, so feeds ran on into the next ball (5 Nov, 084355-092330). After the pulse the feeder holds backward
  // up to AIMED_FEED_PAUSE_S for the dip of a ball already on its way
  public static double AIMED_FEED_PULSE_S = 0.30;
  public static double AIMED_FEED_PAUSE_S = 0.35; // At 0.25 s the third ball's dip came just after the pause (6 Nov, 080814)
  // The wheels lose 300-430 RPM per ball and need about this long to come back (5 Nov, 084355: refed 0.08 s on)
  public static double MIN_BALL_GAP_S = 0.35;
  // From each aimed feed command until this long after, the drive keeps the translation it had at the feed, so the
  // ball leaves at the speed it was aimed for. A ball fed standing still left 0.36 s later with the robot at
  // 0.73 m/s and turning 44 deg/s after the stick was pushed, and missed right (5 Nov, 084758)
  public static double FEED_COMMIT_S = 0.4;
  // Feeds back to back kept renewing the hold, so the robot drove on after the stick was let go and ran into the
  // fence (5 Nov, 090446). Letting go of the stick always stops it, and one hold lasts at most this long
  public static double FEED_COMMIT_MAX_S = 1.0;
  // The commit holds the stick, not the speed: a ball fed in the loop the stick was pushed left with the robot at
  // 0.56 m/s, the wheels 250 RPM fast for that speed, and went long (5 Nov, 085458). So a feed also waits for the
  // drive command to hold within FIRE_STICK_CHANGE for FIRE_STEADY_STICK_S, and for the robot to stop speeding up
  public static double FIRE_STEADY_STICK_S = 0.3;
  public static double FIRE_STICK_CHANGE = 0.1;
  public static double MAX_ACCEL_TO_FIRE_MPS2 = 1.0; // Over ACCEL_WINDOW_S; 8% of moving READY time in soak 081210
  // Over 0.1 s the Pinpoint's speed noise alone read past 1 m/s^2: driving at 0.6+ m/s on a stick held still 0.6 s,
  // 12% of loops over it (median 0.54), and the gate held 40-46% of RT time at 0.6-1.2 m/s. Over 0.2 s, 2% (13 Nov)
  public static double ACCEL_WINDOW_S = 0.2;
  // A feed under way also stops at once, ball out or not, if the hood falls this far off its target (twice the
  // fire check's): the target moves when the robot sets off, and the ball then leaves at the old angle
  public static double FEED_STOP_HOOD_DEG = 2.0;
}
