package org.firstinspires.ftc.teamcode.shooting;

import com.bylazar.configurables.annotations.Configurable;

/**
 * When an aimed shot fires and how its feed runs, one ball at a time (VolleyConstants for bursts). The tolerances
 * match tools/shots/inputs.json. Editable live from Panels.
 */
@Configurable
public final class FireConstants {
  private FireConstants() {}

  // Per flywheel. The hub's speed reading steps by about 61 RPM and the wheels settle a step or so above
  // target, so this allows two steps
  public static double RPM_TOLERANCE = 125;
  // The two wheels' mean, which the aimed shot also needs. The shot table only scores within 50 RPM either side
  // (tools/shots/inputs.json rpmToleranceRpm), and 50 RPM is about 0.1 m of range, so neither side may go past it
  public static double RPM_MEAN_ABOVE_TARGET = 50;
  public static double RPM_MEAN_BELOW_TARGET = 50;
  // The speed this far ahead, from its trend, must not be more than this over: wheels climbing back after a ball
  // can be fed near target and overshoot, sending the ball long. Fast side only, and looser than the speed now:
  // wheels slowing toward the target settle rather than undershoot. The mean steps 30 RPM, so the slope needs 0.2 s
  public static double RPM_PREDICTED_ABOVE_TARGET = 100;
  public static double RPM_LOOKAHEAD_S = 0.08;
  public static double RPM_TREND_WINDOW_S = 0.2;
  public static boolean PRESPIN_ON = true; // Flywheels track the aimed RPM whenever the Cell is known
  public static double PRESPIN_HOLD_S = 1.5; // Keeps that speed this long after the shot goes invalid
  public static double HOOD_TOLERANCE_DEG = 1.0;
  // Otherwise it grows large at short range. The geometric clearance leaves nothing for the other errors: every
  // moving ball that scored left within 1.1 degrees, the misses far wider and all the way the robot moved. 4 keeps
  // most of moving READY time
  public static double MAX_HEADING_TOLERANCE_DEG = 4;
  // No shot while moving at least MOVING_SHOT_SPEED_MPS this close to the lip, nor while the hood's target swings
  // faster than MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC. Balls fed READY while strafing close past the Cell missed: during
  // the 0.3 s feed the heading error grew and the hood target moved. Costs about a quarter of moving READY time
  public static double MOVING_SHOT_SPEED_MPS = 0.3;
  public static double MIN_MOVING_SHOT_LIP_M = 0.7;
  public static double MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = 12;
  // How long an aimed shot must stay ready before the feed starts. Every check above already has to pass,
  // and waiting two or three loops for it to repeat cost more shots than it saved while moving
  public static double AIMED_READY_S = 0.0;
  // A feed with no ball out after this long stops once the shot is no longer ready. Past FEED_DELAY_MAX_S, so a
  // ball already on its way isn't cut off
  public static double AIMED_FEED_STOP_UNREADY_S = 0.4;
  // The longest an aimed feed runs before a ball is seen leaving. The wheel speed shows a ball 0.1-0.17 s late, so
  // longer feeds ran on into the next ball. After the pulse the feeder holds backward up to AIMED_FEED_PAUSE_S for
  // the dip of a ball already on its way
  public static double AIMED_FEED_PULSE_S = 0.30;
  public static double AIMED_FEED_PAUSE_S = 0.35; // Shorter, a ball's dip can come just after the pause
  // The wheels lose 300-430 RPM per ball and need about this long to come back
  public static double MIN_BALL_GAP_S = 0.35;
  // From each aimed feed command until this long after, the drive keeps the translation it had at the feed, so the
  // ball leaves at the speed it was aimed for. A ball fed standing still otherwise left with the robot moving and
  // turning after the stick was pushed, and missed
  public static double FEED_COMMIT_S = 0.4;
  // Feeds back to back kept renewing the hold, so the robot drove on after the stick was let go and ran into the
  // fence. Letting go of the stick always stops it, and one hold lasts at most this long
  public static double FEED_COMMIT_MAX_S = 1.0;
  // The commit holds the stick, not the speed: a ball fed in the loop the stick was pushed left with the robot
  // moving, the wheels fast for that speed, and went long. So a feed also waits for the drive command to hold
  // within FIRE_STICK_CHANGE for FIRE_STEADY_STICK_S, and for the robot to stop speeding up
  public static double FIRE_STEADY_STICK_S = 0.3;
  public static double FIRE_STICK_CHANGE = 0.1;
  public static double MAX_ACCEL_TO_FIRE_MPS2 = 1.0; // Over ACCEL_WINDOW_S
  // Over 0.1 s the Pinpoint's speed noise alone often read past 1 m/s^2 driving on a stick held still, and the gate
  // held much of RT time. Over 0.2 s it rarely does
  public static double ACCEL_WINDOW_S = 0.2;
  // A feed under way also stops at once, ball out or not, if the hood falls this far off its target (twice the
  // fire check's): the target moves when the robot sets off, and the ball then leaves at the old angle
  public static double FEED_STOP_HOOD_DEG = 2.0;
}
