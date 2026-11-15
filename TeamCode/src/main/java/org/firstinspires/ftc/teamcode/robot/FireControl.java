package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.FireConstants;
import org.firstinspires.ftc.teamcode.shooting.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooting.ShooterTuning;
import org.firstinspires.ftc.teamcode.shooting.feed.LoadingShooterInterlock;
import org.firstinspires.ftc.teamcode.shooting.feed.ManualFeedGate;
import org.firstinspires.ftc.teamcode.shooting.feed.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.shooting.feed.RpmTrend;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodGeometry;
import org.psilynx.psikit.core.Logger;

/**
 * When a ball may go: whether the wheels, hood, heading and motion are ready for this loop's shot, and the feed
 * gates that turn that into one aimed ball at a time, a volley's burst, or a manual shot without the Cell.
 */
final class FireControl {
  final LoadingShooterInterlock loadingShooter = new LoadingShooterInterlock();
  private final ManualFeedGate manualFeedGate = new ManualFeedGate();
  // An aimed shot feeds one ball at a time, each once the wheels are back up to speed
  private final OneBallFeedGate aimedFeedGate = new OneBallFeedGate(
      FireConstants.AIMED_READY_S, FireConstants.AIMED_FEED_STOP_UNREADY_S,
      FireConstants.AIMED_FEED_PULSE_S, FireConstants.AIMED_FEED_PAUSE_S,
      FireConstants.MIN_BALL_GAP_S);
  final OneBallFeedGate volleyFeedGate = new OneBallFeedGate(0.0,
      FireConstants.AIMED_FEED_STOP_UNREADY_S, VolleyConstants.VOLLEY_FEED_PULSE_S,
      VolleyConstants.VOLLEY_FEED_PAUSE_S,
      VolleyConstants.VOLLEY_MIN_GAP_S, true);
  OneBallFeedGate feedGate = aimedFeedGate; // The one feeding this loop
  boolean volleyShot = false; // This loop's shot is a volley
  final RpmTrend rpmTrend = new RpmTrend(FireConstants.RPM_TREND_WINDOW_S);
  // This loop's results: checkReady sets the first four for updateFeed, which runs after the drive
  boolean ready;
  private boolean motionSteady;
  private boolean hoodReady;
  private boolean shotLost;
  boolean aimedFeed; // The aimed or volley gate feeds this loop
  boolean manualReady; // The manual gate, without the Cell, feeds this loop

  private final Robot robot;

  FireControl(Robot robot) {
    this.robot = robot;
  }

  /** Picks this loop's feed gate, and tells the loading interlock whether a shot is asked for. */
  void selectGate(Aim aim) {
    volleyShot = robot.shooterControl.volleyAim(aim);
    feedGate = volleyShot ? volleyFeedGate : aimedFeedGate;
    loadingShooter.update(robot.clock.seconds(), robot.spinOnlyOnFire, robot.shotRequested() && !robot.holdFire,
        robot.shooter.getMaxMeasuredRpm());
    Logger.recordOutput("Shooter/Loading/State", loadingShooter.state());
    Logger.recordOutput("Shooter/Loading/SpinBlocked", loadingShooter.blocksSpin());
    Logger.recordOutput("Shooter/Loading/CanLoad", loadingShooter.canLoad());
  }

  /** Whether a ball fed now should score: the shot and heading, the robot's motion, the wheels and the hood. */
  boolean checkReady(boolean aiming, Aim aim, FieldPose pose, double[] velocity) {
    rpmTrend.update(robot.clock.seconds(), robot.shooter.getRpm());
    double predictedRpmError = rpmTrend.predicted(FireConstants.RPM_LOOKAHEAD_S) - robot.shooter.getTargetRpm();
    Logger.recordOutput("Shooter/PredictedRpmError", predictedRpmError);
    boolean flywheelsReady = robot.shooter.atSpeed(FireConstants.RPM_TOLERANCE)
        && robot.shooter.meanRpmWithin(FireConstants.RPM_MEAN_BELOW_TARGET,
            FireConstants.RPM_MEAN_ABOVE_TARGET)
        && predictedRpmError <= FireConstants.RPM_PREDICTED_ABOVE_TARGET;
    if (volleyShot) {
      // Every ball at the same speed, so their flights match and none catches the one ahead
      flywheelsReady = robot.shooter.atSpeed(FireConstants.RPM_TOLERANCE)
          && robot.shooter.meanRpmWithin(VolleyConstants.VOLLEY_READY_BELOW_RPM, volleyReadyAboveRpm(aim))
          && predictedRpmError >= -VolleyConstants.VOLLEY_READY_BELOW_RPM
          && predictedRpmError <= volleyReadyAboveRpm(aim);
    }
    // A volley's hood stays put between balls, so after the first it needn't wait to settle
    hoodReady = volleyShot && volleyFeedGate.ballsLeft() > 0
        ? robot.hood.atTarget(volleyHoodToleranceTicks()) : robot.hood.atTargetSteady(hoodToleranceTicks());
    motionSteady = robot.driveControl.updateMotionSteady(aiming, velocity);
    ready = aim != null && motionSteady && ShootOnTheMove.readyToFire(aim, flywheelsReady, hoodReady,
        pose.headingRad, velocity[2], robot.driveControl.turningRateRadPerSec, robot.closeMovingGateBypassed);
    // What READY waits on, one by one: the shot and heading alone, then the wheels and the hood
    Logger.recordOutput("SOTM/Gate/AimReady", aim != null && ShootOnTheMove.readyToFire(aim, true, true,
        pose.headingRad, velocity[2], robot.driveControl.turningRateRadPerSec, robot.closeMovingGateBypassed));
    Logger.recordOutput("SOTM/Gate/CloseMoving", aim != null && ShootOnTheMove.tooCloseToShootMoving(aim.shot));
    Logger.recordOutput("SOTM/Gate/CloseMovingBypassed", robot.closeMovingGateBypassed);
    Logger.recordOutput("SOTM/Gate/Flywheels", flywheelsReady);
    Logger.recordOutput("SOTM/Gate/Hood", hoodReady);
    // A feed under way stops if the shot goes bad before the ball is out: the driver set off or turned back
    // mid-feed, and the ball left 3 degrees of hood or 37 degrees of heading off (5 Nov, 095605)
    shotLost = aim != null && (!aim.shot.valid
        || Math.abs(Angles.normalize(aim.lockHeadingRad - pose.headingRad)) > aim.headingToleranceRad
        || !robot.hood.atTarget(feedStopHoodTicks()));
    return ready;
  }

  /** Runs the feed gates for this loop's shot, setting {@link #aimedFeed} and {@link #manualReady}. */
  void updateFeed(boolean shootAll, boolean oneBall, Aim aim, FieldPose pose, double[] velocity) {
    // Without the Cell in view there is nothing to aim at, so RT shoots at full flywheel speed and the
    // held hood once the wheels are up to speed, instead of holding the feeder backward forever. It then keeps
    // feeding until RT is released: each ball drops the wheels' RPM, and reversing the feeder every
    // time they dip would pull the next ball back out.
    // Never in the Scoring Auto: with no Cell there is nothing to aim at, and a blind shot is a wasted ball
    manualReady = manualFeedGate.update(shootAll && aim == null && !robot.tuningMode && !robot.autonomous
        && !robot.shooter.isStalled() && !robot.holdFire && !loadingShooter.blocksSpin(),
        robot.shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE));
    // One ball also without the Cell in view, at full speed, so a press never empties the robot
    boolean oneBallReady = aim != null ? ready
        : !robot.tuningMode && robot.shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE) && !robot.shooter.isStalled();
    // Held fire counts as released: the gate cycled through its pulses with the feeder held, so the first ball
    // got the last 0.06 s of a pulse and fell short (6 Nov, 081518)
    volleyFeedGate.setLeadBelowRpm(robot.autonomous ? VolleyConstants.AUTO_VOLLEY_LEAD_BELOW_RPM
        : VolleyConstants.VOLLEY_LEAD_BELOW_RPM);
    volleyFeedGate.setMinFeedSpacingS(VolleyConstants.VOLLEY_MIN_FEED_SPACING_S);
    boolean readyButWheels = aim != null && motionSteady && hoodReady && ShootOnTheMove.readyToFire(aim, true,
        hoodReady, pose.headingRad, velocity[2], robot.driveControl.turningRateRadPerSec,
        robot.closeMovingGateBypassed);
    aimedFeed = feedGate.update(robot.clock.seconds(),
        ((shootAll && aim != null) || oneBall) && !robot.holdFire && !loadingShooter.blocksSpin(),
        oneBall ? oneBallReady : ready, robot.shooter.getRpm(), robot.shooter.getTargetRpm(), oneBall, shotLost,
        readyButWheels && !robot.shooter.isStalled());
    if (robot.volleyMode) { // The other gate starts afresh when the shot switches between volley and one ball
      (volleyShot ? aimedFeedGate : volleyFeedGate).update(robot.clock.seconds(), false, false, robot.shooter.getRpm(),
          robot.shooter.getTargetRpm());
    }
    Logger.recordOutput("Feeder/Gate", feedGate.stateName());
    Logger.recordOutput("Feeder/BallLeft", feedGate.ballLeft());
  }

  private static int feedStopHoodTicks() {
    return (int) Math.round(FireConstants.FEED_STOP_HOOD_DEG * HoodGeometry.TICKS_PER_DEG);
  }

  private static int volleyHoodToleranceTicks() {
    return (int) Math.round(VolleyConstants.VOLLEY_HOOD_TOLERANCE_DEG * HoodGeometry.TICKS_PER_DEG);
  }

  private static int hoodToleranceTicks() {
    return (int) Math.round(FireConstants.HOOD_TOLERANCE_DEG * HoodGeometry.TICKS_PER_DEG);
  }

  /** Top of a volley's matched band: tighter for a backward burst, which goes long when fed fast. */
  private static double volleyReadyAboveRpm(Aim aim) {
    return aim != null && aim.backward ? VolleyConstants.VOLLEY_BACKWARD_READY_ABOVE_RPM
        : VolleyConstants.VOLLEY_READY_ABOVE_RPM;
  }
}
