package org.firstinspires.ftc.teamcode.shooting.volley;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Volleys: Lebot Teleop RB, RT while VOLLEY_ON_RT, and Volley Tuning. A volley is RT's aimed hooded shot fired as a
 * burst: each ball goes as soon as the wheels are back at the matched speed, instead of FireConstants' one-ball pulse,
 * pause and gap. One flywheel can't launch four balls at the same instant; the burst is limited by the wheels'
 * recovery between balls.
 *
 * <p>Balls in a burst must leave at the same wheel speed and hood. Modelled (tools/shots, collide.js), two balls 0.1 s
 * apart at matched speed stay 0.12-0.18 m apart in flight (ball 0.071 m), but a second ball 200-300 RPM slower lands
 * first and the paths cross within 0.01-0.05 m. So each ball waits for the matched band below, and the feed power sets
 * how tightly the queue follows the ball ahead. Editable live from Panels.
 */
@Configurable
public final class VolleyConstants {
  private VolleyConstants() {}

  // RPM added to the table's for a volley, with the hood steepened for it (ShotTableRpm). Boost doesn't widen the
  // scoring room: modelled at 32 cells it is 75-125 RPM either side at +0, 25 above at +150, 75 either side at +200
  // to +300. So it stays 0 unless Volley Tuning shows the burst needs the headroom
  public static double VOLLEY_BOOST_RPM = 0;
  // How much the hood follows the wheels' predicted speed off the volley target (0 = fixed hood for the burst). Within
  // the band below the steepening is under 1.5 deg, inside HOOD_TOLERANCE_DEG, and the reading lags 100-250 RPM at
  // full-power recovery, so following would chase noise and move the hood between balls
  public static double VOLLEY_HOOD_FOLLOW_GAIN = 0;
  // Matched band: each ball feeds only with the wheels' mean and their speed RPM_LOOKAHEAD_S ahead this close to
  // the volley target. Below is RT's 100: the hub's speed control sits 50-80 RPM under the target at rest, so 75
  // flickered in and out, and in Volley Tuning balls fed 50-100 under went in as often as balls fed on it (12 Nov)
  public static double VOLLEY_READY_BELOW_RPM = 100;
  public static double VOLLEY_READY_ABOVE_RPM = 75;
  // Backward shots burst too (13 Nov, for the Hive-front wedge). Shot Tuning's 138 standing backward balls dipped the
  // wheels as hooded ones do (median 307 RPM, p90 369 against 307/400 for 225 hooded), so the recovery and the
  // double-ball check carry over. But fed above the table's speed they go long: hood within 1.5 deg of the table,
  // fed -50..0 RPM scored 14 of 19, -100..-50 6 of 12, 0..+50 3 of 8 (3 long), +50..+100 2 of 5 (3 long). So the
  // band's top is 25 for a backward burst; its bottom stays VOLLEY_READY_BELOW_RPM (the wheels sit 50-80 under at
  // rest). No boost and no hood trims: the backward hood follows its own table
  public static boolean VOLLEY_BACKWARD_ON = true;
  public static double VOLLEY_BACKWARD_READY_ABOVE_RPM = 25;
  // Volley Tuning's backward spots search the same trim as the hooded ones, but a backward ball's range is set by the
  // RPM (Shot Tuning searches it in 100 RPM steps to 25). Each degree of trim is this many RPM, less RPM for more
  // trim as a steeper hooded ball goes shorter: the search's 1 deg step is 50 RPM, its 0.5 resolution 25, its
  // +-4 deg limit +-200
  public static double VOLLEY_BACKWARD_RPM_PER_TRIM_DEG = 50;
  // A backward volley's trims without a confirmed backward spot, from Volley Tuning's B3 (15 Nov, 30 volleys): first
  // balls at -3.5..-4.0 went in 7 of 9 against 2 of 6 at -2.7..-3.0 (mostly short); later balls 4 of 4 at -1.5 but
  // 20 of 34 at -0.2..-0.4 with 11 short, so between those
  public static double VOLLEY_BACKWARD_FIRST_BALL_TRIM_DEG = -3.7;
  public static double VOLLEY_BACKWARD_LATER_BALL_TRIM_DEG = -1.0;
  // Launch angle added for the first ball and for balls 2-4, used only without Volley Tuning's CSV. The medians of
  // the eight forward spots confirmed by 15 Nov 1140 (first -2.4..+0.9, later -1..0)
  public static double VOLLEY_FIRST_BALL_TRIM_DEG = -0.75;
  public static double VOLLEY_LATER_BALL_TRIM_DEG = -0.75;
  // Lebot Teleop and the scoring auto read Volley Tuning's CSV for their volley settings (VolleySettings), from its
  // confirmed spots only. Back on 15 Nov: spots 4-10 are confirmed at today's full feed, 80-90% in; it was off from
  // 12 Nov while the CSV held only the old ringing recovery and slow feed, whose spot 0 is still unconfirmed
  public static boolean VOLLEY_USE_TUNING_CSV = true;
  // Lebot Teleop's RT shoot on the move fires its aimed hooded shots as a volley too, with RB's feed, recovery and
  // matched band; the robot keeps moving and the aim keeps leading it. False: RT one ball at a time as before
  public static boolean VOLLEY_ON_RT = true;
  // Feeder power while a volley ball feeds. Full, as RT's shot: the ball the feeder has moves away from the next
  // one, which the indexer's last roller brings up at VOLLEY_INDEXER_SCALE of it, so the gap opens 3x as fast as
  // the queue closes it. At 0.6 the feed took 0.21-0.24 s a ball, and 0.6/0.3 left the 4th ball stuck (12 Nov)
  public static double VOLLEY_FEED_POWER = 1.0;
  // Indexer power while a ball feeds, as a share of the feeder's. Between balls the indexer stages the rest of the
  // stack at full power against its stopped last roller, so the next ball is always there (it took 2.9 s to come
  // up at 0.3, 12 Nov 211957). Volley Tuning at 0.74 m (14 Nov 224053): full feeder and indexer 8 of 8 in, every
  // pulse a ball, the last out 1.2 s after the first feed; indexer at 0.3 (feeder 0.6) 8 of 8 but a pulse in one
  // volley came up empty and the four took 1.7-2.2 s; at 0.15, 3.3-4.9 s. The auto already runs full
  public static double VOLLEY_INDEXER_SCALE = 1.0;
  // Shortest time from a ball seen leaving to the next feed. The wheels' matched band is the main wait
  public static double VOLLEY_MIN_GAP_S = 0.05;
  // Shortest time from one volley feed to the next. Volley Tuning's profile 0 fed 0.30-0.37 s apart and went 8 of 8
  // (14 Nov 224053); TeleOp's stack, pushed up by loading, fed 0.12 and 0.20 s apart (230333): two balls out
  // 0.12 s apart, into each other in the Cell
  public static double VOLLEY_MIN_FEED_SPACING_S = 0.30;
  // The next ball of a volley starts once the wheels are back within this of the target and rising, instead of
  // inside the band: the ball's 0.26 s to the wheels then overlaps their recovery. Replaying 13 Nov's gaps saved
  // 0.18 s a ball (0.58 s each); the slowest tenth of balls leave ~70 under target against ~35. 0 waits for the band
  public static double VOLLEY_LEAD_BELOW_RPM = 150;
  // The auto's: its last roller keeps loading between balls, so the next is already at the wheels and the lead's
  // 0.26 s isn't there. With 150 the 4th preload left 0.17-0.20 s after the 3rd, wheels ~120 under, and missed in
  // every corner auto (15 Nov 153408, 154350); Volley Tuning's were ~0.34 s apart
  public static double AUTO_VOLLEY_LEAD_BELOW_RPM = 0;
  // As AIMED_FEED_PULSE_S and AIMED_FEED_PAUSE_S, for a volley
  public static double VOLLEY_FEED_PULSE_S = 0.30;
  public static double VOLLEY_FEED_PAUSE_S = 0.20;
  // Between balls of a volley the feeder runs backward, as before the first ball, so every ball starts from the
  // same place as RT's shot and none creeps into the wheels while they recover (was false: stopped)
  public static boolean VOLLEY_HOLD_BETWEEN_BALLS = true;
  // Hood within this of its target for balls after the first, without the steady wait (it isn't moving)
  public static double VOLLEY_HOOD_TOLERANCE_DEG = 1.0;
  // Recovery after each ball: open loop this far below target, speed control again within the handback. The speed
  // reading lags ~0.1 s, and full power gains ~5500 RPM/s, so the wheels run on ~500 RPM past the handback point.
  // At 200/100 that overshot to +600, dipped below 200 again and rang +-500 RPM for ~0.8 s every ball, which was most
  // of the volley's time, and the swings read as double balls (12 Nov, 211957). Speed control alone recovers a 300-
  // 430 RPM dip to the band in ~0.25 s (9 Nov TeleOp); only a deeper dip gets full power, handed back 450 short
  public static boolean VOLLEY_FAST_RECOVERY = true;
  public static double VOLLEY_OPEN_LOOP_BELOW_RPM = 600;
  public static double VOLLEY_HANDBACK_RPM = 450;
  // A ball's dip deeper than this (peak before it to the trough after) is taken as two balls leaving together. A
  // single ball dropped 300-430 RPM under speed control (5 Nov)
  public static double VOLLEY_DOUBLE_DIP_RPM = 600;
}
