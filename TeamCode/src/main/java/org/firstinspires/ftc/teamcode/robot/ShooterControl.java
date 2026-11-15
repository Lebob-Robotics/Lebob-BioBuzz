package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.control.HoodStepControl;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.shooting.BackwardShotConstants;
import org.firstinspires.ftc.teamcode.shooting.FireConstants;
import org.firstinspires.ftc.teamcode.shooting.Shot;
import org.firstinspires.ftc.teamcode.shooting.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooting.ShooterTuning;
import org.firstinspires.ftc.teamcode.shooting.SotmConstants;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodGeometry;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodSubsystem.HoodPreset;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;

/**
 * The flywheels' and hood's targets each loop: the shot's speed and launch angle with the moving-shot corrections,
 * the volley's boost and trims, prespin between shots, and the driver's full power, eject, stop and presets.
 */
final class ShooterControl {
  private final Robot robot;
  private double aimRpm = DriverConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private final HoodStepControl hoodSteps = new HoodStepControl();
  double volleyHoodDeg = 0; // Launch angle the volley added this loop, for the log
  private boolean shooterStopLatched = false; // X: flywheels off until the driver next asks for a shot
  private double lastShotKnownS = Double.NEGATIVE_INFINITY;

  ShooterControl(Robot robot) {
    this.robot = robot;
  }

  /** The flywheels' state and speed and the hood's target this loop, for the aim (null: no Cell known). */
  void update(boolean aiming, Aim aim) {
    volleyHoodDeg = 0;
    boolean validShot = aim != null && aim.shot.valid;
    // A backward shot waiting only for the robot to slow down: hood down and wheels at its speed already
    boolean backwardWaiting = aim != null && aim.backward && !validShot
        && aim.shot.reason != Shot.Reason.OUT_OF_RANGE;
    boolean shotKnown = validShot || backwardWaiting;
    double nowS = robot.clock.seconds();
    if (shotKnown) { lastShotKnownS = nowS; }
    // A shot gone for a moment (NO_SAFE_SHOT flickers) keeps its speed: dropping to IDLE braked the wheels, and they
    // then had to come back
    boolean shotJustLost = !shotKnown && nowS - lastShotKnownS <= FireConstants.PRESPIN_HOLD_S;
    double rpmRate = shotKnown ? aim.rpmRatePerSec : 0.0;
    // The shot is for the ball leaving FEED_DELAY_S from now. The wheels and hood only have to be there by
    // then, so they follow the shot for now and arrive at this one as the ball leaves. Holding this one
    // already put them a feed delay early.
    if (shotKnown && !Double.isNaN(aim.rpmNow)) {
      aimRpm = aim.rpmNow;
    }

    boolean fullPower = robot.driver.y && robot.buttonsFree();
    boolean eject = robot.driver.b && robot.buttonsFree();
    if (aiming || fullPower || eject) {
      shooterStopLatched = false;
    } else if (robot.driver.x && robot.buttonsFree()) {
      shooterStopLatched = true;
    }
    if (robot.volleyMode) { // Full power back up to speed after each ball of a volley
      boolean fast = robot.fireControl.volleyShot && robot.volley.fastRecovery();

      robot.shooter.setSpinThresholds(fast ? VolleyConstants.VOLLEY_OPEN_LOOP_BELOW_RPM : Double.NaN,
          fast ? VolleyConstants.VOLLEY_HANDBACK_RPM : Double.NaN);
    }

    if (eject) {
      robot.shooter.setShooterState(ShooterState.EJECT);
    } else if (robot.fireControl.loadingShooter.blocksSpin()) {
      // Heading/hood still track below; no powered idle while filling.
      robot.shooter.setShooterState(ShooterState.STOP);
    } else if (aiming) {
      robot.shooter.setShooterState(ShooterState.AIM);
      // No Cell in view means no shot table to read, so go for the top speed
      robot.shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0
          ? ShooterTuning.RPM_OVERRIDE
          : aim == null ? robot.shooter.getMaxRpm()
          : aimRpm + robot.rpmTrim + tableRpmOffset(aim) + volleyBoostRpm(aim),
          ShooterTuning.RPM_OVERRIDE > 0 ? 0.0 : rpmRate);
    } else if (fullPower) {
      if (ShooterTuning.RPM_OVERRIDE > 0) {
        robot.shooter.setShooterState(ShooterState.AIM);
        robot.shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE);
      } else {
        robot.shooter.setShooterState(ShooterState.SHOOT);
      }
    } else if (shooterStopLatched || robot.tuningMode) { // The tuning OpModes rest the wheels between shots
      robot.shooter.setShooterState(ShooterState.STOP);
    } else if (shotKnown && FireConstants.PRESPIN_ON) {
      // Already at the speed RT would ask for, so a shot doesn't wait on spin-up (0.8 s from idle)
      robot.shooter.setShooterState(ShooterState.AIM);
      robot.shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0 ? ShooterTuning.RPM_OVERRIDE
          : aimRpm + robot.rpmTrim + tableRpmOffset(aim) + volleyBoostRpm(aim) + volleyBackwardRpm(aim),
          ShooterTuning.RPM_OVERRIDE > 0 ? 0.0 : rpmRate);
    } else if (shotJustLost && FireConstants.PRESPIN_ON
        && robot.shooter.getShooterState() == ShooterState.AIM) {
      robot.shooter.setShooterState(ShooterState.AIM); // At the last target, which it keeps
    } else {
      robot.shooter.setShooterState(ShooterState.IDLE);
    }

    // Not aiming but the Cell is known: move the hood to the shot RT would take now, while driving,
    // so it is already there when RT is pressed. Presets are for when the Cell isn't known.
    boolean prepositioning = !aiming && shotKnown && !fullPower && !eject;
    if (aiming || prepositioning) {
      if (robot.tuningMode && !robot.tuningLive && validShot) {
        // Lift 0 is the lower stop; held just above it, as a backward shot's is, so the hood can't sag
        robot.hood.setTargetTicks(Math.max(HoodSubsystem.minTargetTicks(), HoodSubsystem.angleToTicks(aim.hoodNowDeg)));
      } else if (shotKnown && aim.backward) {
        // Never under just above the lower stop. Target 0 cuts the motor to rest on the stop, but the hood
        // doesn't fall by itself: it stayed up and the shot never got ready. Trim doesn't apply
        robot.hood.setTargetTicks(Double.isNaN(aim.hoodNowDeg) ? HoodSubsystem.minTargetTicks()
            : Math.max(HoodSubsystem.minTargetTicks(), HoodSubsystem.angleToTicks(aim.hoodNowDeg)));
      } else if (validShot) { // Otherwise the hood holds its last angle
        // Lift rises as the launch angle falls
        volleyHoodDeg = volleyHoodDeg(aim);
        robot.hood.setTargetTicks(HoodSubsystem.angleToTicks(aim.hoodNowDeg - robot.hoodLiftTrimDeg + movingHoodDeg(aim)
            + volleyHoodDeg), -aim.hoodRateDegPerSec);
      }
    } else if (!robot.buttonsFree()) {
      return; // The D-pad sets the tuning shot, or marks balls, instead
    } else if (robot.driver.dpad_up) {
      robot.hood.setPreset(HoodPreset.FAR);
    } else if (robot.driver.dpad_down) {
      robot.hood.setPreset(HoodPreset.CLOSE);
    }
    if (!robot.buttonsFree()) {
      return;
    }
    double lift = robot.hood.getTargetLiftDegrees();
    double steppedLift = hoodSteps.update(lift, robot.driver.dpad_right, robot.driver.dpad_left,
        HoodGeometry.ZERO_ANGLE_DEG - HoodGeometry.MIN_ANGLE_DEG);
    if (steppedLift != lift) {
      if (aiming || prepositioning) { robot.hoodLiftTrimDeg += steppedLift - lift; }
      robot.hood.setTargetLiftDegrees(steppedLift);
    }
  }

  double tableRpmOffset(Aim aim) {
    if (aim == null || (robot.tuningMode && !robot.tuningLive)) {
      return 0;
    }
    double toward = finiteOr0(aim.shot.speedTowardCellMps);
    return aim.backward ? SotmConstants.SOTM_BACKWARD_RPM_PER_TOWARD_MPS * toward
        : AimConstants.HOODED_RPM_OFFSET + SotmConstants.SOTM_RPM_PER_TOWARD_MPS * toward;
  }

  /** Launch angle added to a hooded shot for its speed toward the Cell. */
  private double movingHoodDeg(Aim aim) {
    if (aim == null || aim.backward || (robot.tuningMode && !robot.tuningLive)) {
      return 0;
    }
    double towardMps = finiteOr0(aim.shot.speedTowardCellMps);
    return SotmConstants.SOTM_HOOD_DEG_PER_TOWARD_MPS * towardMps + (robot.tuningMode ? 0
        : movingHoodTrimDeg(aim.shot.distanceToLipM, Math.hypot(towardMps, finiteOr0(aim.shot.sidewaysSpeedMps))));
  }

  /**
   * TeleOp's launch angle added to a moving hooded shot for its distance to the lip
   * ({@link SotmConstants#SOTM_HOOD_TRIM_DEG}): full from the moving shot speed, scaled down to none at rest.
   */
  static double movingHoodTrimDeg(double lipM, double speedMps) {
    double[] d = SotmConstants.SOTM_HOOD_TRIM_DIST_M;
    double[] t = SotmConstants.SOTM_HOOD_TRIM_DEG;
    if (Double.isNaN(lipM) || Double.isNaN(speedMps)) {
      return 0;
    }
    double trim;
    if (lipM <= d[0]) {
      trim = t[0];
    } else if (lipM >= d[d.length - 1]) {
      trim = t[t.length - 1];
    } else {
      int i = 0;
      while (lipM > d[i + 1]) { i++; }
      trim = t[i] + (t[i + 1] - t[i]) * (lipM - d[i]) / (d[i + 1] - d[i]);
    }
    return trim * Math.min(1, Math.abs(speedMps) / FireConstants.MOVING_SHOT_SPEED_MPS);
  }

  /**
   * Aim trim, degrees anticlockwise: the tuning tools' own (toolTrimDeg), else the standing trim they found for the
   * hooded or backward shot.
   */
  static double aimTrimDeg(boolean tuningMode, double toolTrimDeg, boolean backward) {
    if (tuningMode) {
      return toolTrimDeg;
    }
    return backward ? BackwardShotConstants.BACKWARD_HEADING_TRIM_DEG
        : AimConstants.HOODED_HEADING_TRIM_DEG;
  }

  /** Aim turned anticlockwise for the shot's sideways speed. */
  double movingHeadingDeg(Aim aim) {
    if (aim == null || (robot.tuningMode && !robot.tuningLive)) {
      return 0;
    }
    return (aim.backward ? SotmConstants.SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS
        : SotmConstants.SOTM_HEADING_DEG_PER_SIDEWAYS_MPS) * finiteOr0(aim.shot.sidewaysSpeedMps);
  }

  private static double finiteOr0(double v) {
    return Double.isNaN(v) || Double.isInfinite(v) ? 0 : v;
  }

  boolean volleyAim(Aim aim) {
    return robot.volleyMode && robot.volleyRequested && aim != null
        && (!aim.backward || VolleyConstants.VOLLEY_BACKWARD_ON);
  }

  private double volleyBoostRpm(Aim aim) {
    return volleyAim(aim) && !aim.backward ? robot.volleyBoostRpm() : 0;
  }

  /**
   * Launch angle a volley adds: the steepening for its boost (ShotTableRpm), a share of the steepening for the
   * wheels' predicted speed off the target (VOLLEY_HOOD_FOLLOW_GAIN), and the trim for the ball about to go.
   */
  private double volleyHoodDeg(Aim aim) {
    if (!volleyAim(aim)) {
      return 0;
    }
    double boost = robot.volleyBoostRpm();
    double lipM = aim.shot.distanceToLipM;
    double towardMps = finiteOr0(aim.shot.speedTowardCellMps);
    double boostDeg = ShootOnTheMove.hoodOffsetForRpmAbove(lipM, towardMps, boost);
    double followDeg = 0;
    double gain = VolleyConstants.VOLLEY_HOOD_FOLLOW_GAIN;
    if (gain != 0) {
      double above = robot.fireControl.rpmTrend.predicted(FireConstants.RPM_LOOKAHEAD_S)
          - robot.shooter.getTargetRpm() + boost;
      above = Math.max(boost - VolleyConstants.VOLLEY_READY_BELOW_RPM,
          Math.min(boost + VolleyConstants.VOLLEY_READY_ABOVE_RPM, above));
      followDeg = gain * (ShootOnTheMove.hoodOffsetForRpmAbove(lipM, towardMps, above) - boostDeg);
    }
    return finiteOr0(boostDeg + followDeg) + volleyTrimDeg(false);
  }

  /** The trim for the ball about to go (VolleyConfig.trimDeg). */
  private double volleyTrimDeg(boolean backward) {
    return robot.volley.trimDeg(robot.fireControl.volleyFeedGate.ballsLeft() == 0, backward);
  }

  /** A backward volley's trim, as wheel speed: its hood follows the table, and its range comes from the RPM. */
  private double volleyBackwardRpm(Aim aim) {
    return volleyAim(aim) && aim.backward
        ? -VolleyConstants.VOLLEY_BACKWARD_RPM_PER_TRIM_DEG * volleyTrimDeg(true) : 0;
  }
}
