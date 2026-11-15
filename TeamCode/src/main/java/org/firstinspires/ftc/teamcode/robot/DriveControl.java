package org.firstinspires.ftc.teamcode.robot;

import java.util.ArrayDeque;
import org.firstinspires.ftc.teamcode.control.HeadingHold;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.robot.Robot.AutoDriver;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.FireConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.psilynx.psikit.core.Logger;

/**
 * How the Robot drives each loop: the driver's sticks or a script's, RB's position hold, the heading lock while
 * aiming and the heading hold otherwise, the Scoring Auto's powers, and whether the drive is steady enough to fire.
 */
final class DriveControl {
  /** Feedforward cap: very close to the Cell the lock heading can swing faster than the robot can turn. */
  private static final double MAX_LOCK_RATE_RAD_S = 3.0;
  /** Share of each loop's new sideways-turn rate taken in: it comes from differenced odometry velocity. */
  private static final double TURNING_RATE_SMOOTHING = 0.5;
  // RB's position hold: field-frame power toward the anchor, less a share of the speed to damp it
  private static final double HOLD_DEADBAND_M = 0.02;
  private static final double HOLD_GAIN_PER_M = 2.5;
  private static final double HOLD_MIN_POWER = 0.08; // Gets the wheels past their friction
  private static final double HOLD_MAX_POWER = 0.4;
  private static final double HOLD_DAMPING_S = 0.3;
  // A scripted hold point rests the wheels near it: holding there, too weak to move the robot, stalled them with
  // a whine. Pushed past RETURN it drives back to DONE, its least power rising until the robot moves
  private static final double HOLD_POINT_RETURN_M = 0.04;
  private static final double HOLD_POINT_DONE_M = 0.015;
  private static final double HOLD_POINT_BOOST_PER_S = 0.3;
  private static final double HOLD_POINT_MOVING_MPS = 0.05;

  private final Robot robot;
  boolean steadyGateBypassed = false; // SOTM Tuning's passes that fire mid-change
  double[] steadyStickReference; // SOTM Tuning: the plan's stick size, without its tracking corrections
  double[] driveOverride = null; // Field-frame stick {+x, +y} for a scripted drive; null for the driver's
  private double holdAnchorX = Double.NaN; // Where RB holds the robot; NaN while not holding
  private double holdAnchorY = Double.NaN;
  double[] holdPoint = null; // A scripted field spot held while no drive is scripted; null for none
  boolean sticksLocked = false; // The drive and turn sticks do nothing
  double turnOverride = 0; // Turn stick for a scripted turn while the sticks are locked
  private boolean holdPointReturning = false;
  private double holdPointStuckSinceS;
  // Rate the aimed shot's sideways turn changes, smoothed, for the fire check and the heading lock
  double turningRateRadPerSec = 0.0;
  private double lastTurningRad = Double.NaN;
  private double lastTurningS = Double.NaN;
  private boolean lastTurningBackward = false;
  final HeadingHold headingHold = new HeadingHold();
  private double feedCommitUntilS = Double.NEGATIVE_INFINITY;
  private double[] committedTranslation; // Held while a fed ball is still in the shooter; null otherwise
  private double committedSinceS;
  private double[] lastTranslation = {0.0, 0.0};
  AutoDriver autoDriver = null;
  /** A heading the auto wants held with the heading lock instead of Pedro's turn, or NaN. */
  double autoHeadingRad = Double.NaN;
  double autoHeadingRateRadPerSec = 0.0;
  private double[] steadyStick = {0.0, 0.0};
  private double steadyStickSinceS = Double.NEGATIVE_INFINITY;
  private final ArrayDeque<double[]> velocityHistory = new ArrayDeque<>(); // {t, x, y}

  DriveControl(Robot robot) {
    this.robot = robot;
  }

  /** The pose was corrected, not moved: RB's held spot moves with it. */
  void shiftHoldAnchor(double dxM, double dyM) {
    holdAnchorX += dxM;
    holdAnchorY += dyM;
  }

  /** RB holds the robot where it was pressed; a scripted hold point holds it there while no drive is scripted. */
  void updateHoldAnchor(boolean holdShot, FieldPose pose) {
    if (!holdShot) {
      holdAnchorX = Double.NaN;
      holdAnchorY = Double.NaN;
    } else if (Double.isNaN(holdAnchorX)) {
      holdAnchorX = pose.xM;
      holdAnchorY = pose.yM;
    }
    if (holdPoint != null && driveOverride == null) {
      holdAnchorX = holdPoint[0];
      holdAnchorY = holdPoint[1];
    }
  }

  /** A ball started to feed: the drive keeps its translation until the ball is out (FEED_COMMIT_S). */
  void commitFeed() {
    feedCommitUntilS = robot.clock.seconds() + FireConstants.FEED_COMMIT_S;
  }

  /** Drives this loop: the auto's powers while an AutoDriver is set, else the sticks or a script. */
  void update(boolean aiming, Aim aim, double lockRateRadPerSec, FieldPose pose, double[] velocity) {
    if (autoDriver != null) {
      driveAuto(aiming, aim, lockRateRadPerSec, pose, velocity);
    } else {
      updateDrive(aiming, aim, lockRateRadPerSec, pose, velocity[2]);
    }
  }

  /**
   * Rate the aimed lock heading moves, CCW positive, for the heading lock's feedforward. 0 when not aiming.
   * The direction to the Cell's rate comes from the velocity; the shooter's sideways turn changes with the
   * acceleration, so its rate is taken loop to loop and smoothed.
   */
  double updateLockRate(boolean aiming, Aim aim) {
    double nowS = robot.clock.seconds();
    if (!aiming || aim == null || aim.backward != lastTurningBackward) {
      turningRateRadPerSec = 0.0;
      lastTurningRad = Double.NaN;
    } else if (!Double.isNaN(lastTurningRad) && nowS - lastTurningS > 1e-3) {
      double rate = (aim.shot.turningRad - lastTurningRad) / (nowS - lastTurningS);
      rate = Math.max(-MAX_LOCK_RATE_RAD_S, Math.min(MAX_LOCK_RATE_RAD_S, rate));
      turningRateRadPerSec += TURNING_RATE_SMOOTHING * (rate - turningRateRadPerSec);
    }
    if (aiming && aim != null) {
      lastTurningRad = aim.shot.turningRad;
      lastTurningS = nowS;
      lastTurningBackward = aim.backward;
    }
    double lockRateRadPerSec = !aiming || aim == null ? 0.0
        : Math.max(-MAX_LOCK_RATE_RAD_S, Math.min(MAX_LOCK_RATE_RAD_S, aim.lockRateRadPerSec + turningRateRadPerSec));
    Logger.recordOutput("Drive/HeadingLock/TargetRateRadPerSec", lockRateRadPerSec);
    Logger.recordOutput("SOTM/TurningRateRadPerSec", turningRateRadPerSec);
    return lockRateRadPerSec;
  }

  /**
   * True once the drive command has held still long enough for the robot to reach its speed, and the robot is no
   * longer speeding up or slowing down: a shot is aimed for the speed now, so it must still be the speed at release.
   */
  boolean updateMotionSteady(boolean aiming, double[] velocity) {
    double nowS = robot.clock.seconds();
    double maxPower = aiming ? DriverConstants.AIM_DRIVE_MAX_POWER : 1.0;
    // Held on the spot the driver asks for no motion; the hold's own corrections show up as acceleration
    double[] stick = isHolding() ? new double[] {0, 0}
        : driveOverride != null && steadyStickReference != null ? steadyStickReference
        : stickTranslation(maxPower);
    if (Math.hypot(stick[0] - steadyStick[0], stick[1] - steadyStick[1]) > FireConstants.FIRE_STICK_CHANGE) {
      steadyStick = stick;
      steadyStickSinceS = nowS;
    }
    velocityHistory.addLast(new double[] {nowS, velocity[0], velocity[1]});
    while (velocityHistory.size() > 2 && nowS - velocityHistory.peekFirst()[0] > FireConstants.ACCEL_WINDOW_S) {
      velocityHistory.removeFirst();
    }
    double[] oldest = velocityHistory.peekFirst();
    double spanS = nowS - oldest[0];
    double accelMps2 = spanS > 0.02 ? Math.hypot(velocity[0] - oldest[1], velocity[1] - oldest[2]) / spanS : 0.0;
    boolean steady = nowS - steadyStickSinceS >= FireConstants.FIRE_STEADY_STICK_S
        && accelMps2 <= FireConstants.MAX_ACCEL_TO_FIRE_MPS2;
    Logger.recordOutput("SOTM/AccelMps2", accelMps2);
    Logger.recordOutput("SOTM/Gate/Steady", steady);
    Logger.recordOutput("SOTM/Gate/SteadyBypassed", steadyGateBypassed);
    return steady || steadyGateBypassed;
  }

  /** {forward, right} for the drive, from the left stick or, while one is set, {@link Robot#setDriveOverride}. */
  private double[] stickTranslation(double maxPower) {
    if (driveOverride == null && sticksLocked) {
      return new double[] {0, 0};
    }
    if (driveOverride == null) {
      return MecanumDriveSubsystem.shapeTranslation(-robot.driver.left_stick_y, robot.driver.left_stick_x, maxPower);
    }
    double f = robot.driverForwardHeadingRad();
    double forward = driveOverride[0] * Math.cos(f) + driveOverride[1] * Math.sin(f);
    double right = driveOverride[0] * Math.sin(f) - driveOverride[1] * Math.cos(f);
    return MecanumDriveSubsystem.shapeTranslation(forward, right, maxPower);
  }

  boolean isHolding() { return !Double.isNaN(holdAnchorX); }

  /** {forward, right} power back to RB's held spot, damped by the speed. Not curved as a stick is. */
  private double[] holdTranslation(FieldPose pose, double minPower) {
    double ex = holdAnchorX - pose.xM;
    double ey = holdAnchorY - pose.yM;
    double away = Math.hypot(ex, ey);
    double vx = -HOLD_DAMPING_S * HOLD_GAIN_PER_M * robot.lastVelocity[0];
    double vy = -HOLD_DAMPING_S * HOLD_GAIN_PER_M * robot.lastVelocity[1];
    if (away > HOLD_DEADBAND_M) {
      double power = Math.min(HOLD_MAX_POWER, minPower + HOLD_GAIN_PER_M * (away - HOLD_DEADBAND_M));
      vx += ex / away * power;
      vy += ey / away * power;
    }
    double magnitude = Math.hypot(vx, vy);
    if (magnitude > HOLD_MAX_POWER) {
      vx *= HOLD_MAX_POWER / magnitude;
      vy *= HOLD_MAX_POWER / magnitude;
    }
    Logger.recordOutput("Drive/PositionHold/ErrorM", away);
    double f = robot.driverForwardHeadingRad();
    return new double[] {vx * Math.cos(f) + vy * Math.sin(f), vx * Math.sin(f) - vy * Math.cos(f)};
  }

  /** {@link #holdTranslation} for a scripted hold point: nothing near it, and back to it once pushed off. */
  private double[] holdPointTranslation(FieldPose pose) {
    double nowS = robot.clock.seconds();
    double away = Math.hypot(holdAnchorX - pose.xM, holdAnchorY - pose.yM);
    if (!holdPointReturning && away > HOLD_POINT_RETURN_M) {
      holdPointReturning = true;
      holdPointStuckSinceS = nowS;
    } else if (holdPointReturning && away < HOLD_POINT_DONE_M) {
      holdPointReturning = false;
    }
    Logger.recordOutput("Drive/PositionHold/Returning", holdPointReturning);
    if (!holdPointReturning) {
      Logger.recordOutput("Drive/PositionHold/ErrorM", away);
      return new double[] {0, 0};
    }
    if (Math.hypot(robot.lastVelocity[0], robot.lastVelocity[1]) > HOLD_POINT_MOVING_MPS) {
      holdPointStuckSinceS = nowS;
    }
    return holdTranslation(pose, HOLD_MIN_POWER + HOLD_POINT_BOOST_PER_S * (nowS - holdPointStuckSinceS));
  }

  private void updateDrive(boolean aiming, Aim aim, double lockRateRadPerSec, FieldPose pose,
      double turnRateRadPerSec) {
    // Slower while shooting, so the shot's lead for the robot's speed stays small and the aim keeps up
    double maxPower = aiming ? DriverConstants.AIM_DRIVE_MAX_POWER : 1.0;
    double[] translation = !isHolding() ? stickTranslation(maxPower)
        : holdPoint != null && driveOverride == null ? holdPointTranslation(pose)
        : holdTranslation(pose, HOLD_MIN_POWER);
    // The drive runs before this loop's feed check, so lastTranslation is what was commanded as the feed started
    double nowS = robot.clock.seconds();
    boolean stickReleased = translation[0] == 0.0 && translation[1] == 0.0;
    if (isHolding()) {
      feedCommitUntilS = Double.NEGATIVE_INFINITY; // The hold keeps it still through the feed already
    }
    if (committedTranslation != null
        && (stickReleased || nowS - committedSinceS > FireConstants.FEED_COMMIT_MAX_S)) {
      feedCommitUntilS = Double.NEGATIVE_INFINITY;
    }
    if (aiming && nowS < feedCommitUntilS) {
      if (committedTranslation == null) {
        committedTranslation = lastTranslation;
        committedSinceS = nowS;
      }
      translation = committedTranslation;
    } else {
      committedTranslation = null;
    }
    lastTranslation = translation;
    Logger.recordOutput("Drive/FeedCommit", committedTranslation != null);
    double forward = translation[0];
    double right = translation[1];
    double forwardOffsetRad = robot.driverForwardHeadingRad();
    double driverHeadingRad = pose.headingRad - forwardOffsetRad;
    if (aiming && aim != null) { // Lines up even without a valid shot, so it's ready the moment one appears
      // Target and heading both less the offset, so the error is the same and the driving is driver relative
      robot.drive.driveHeadingLock(forward, right, aim.lockHeadingRad - forwardOffsetRad, lockRateRadPerSec,
          driverHeadingRad, turnRateRadPerSec);
      headingHold.clear();
    } else {
      double turn = sticksLocked ? turnOverride : MecanumDriveSubsystem.shapeTurnStick(robot.driver.right_stick_x);
      double holdRad = headingHold.update(turn != 0.0, forward != 0.0 || right != 0.0, pose.headingRad,
          turnRateRadPerSec, robot.clock.seconds());
      if (Double.isNaN(holdRad)) {
        robot.drive.drive(forward, right, turn, true, driverHeadingRad);
      } else {
        robot.drive.driveHeadingHold(forward, right, holdRad - forwardOffsetRad, driverHeadingRad, turnRateRadPerSec);
      }
    }
    Logger.recordOutput("Drive/HeadingHold/TargetRad", headingHold.getTargetRad());
  }

  /**
   * Drives with the Scoring Auto's powers. They are robot-frame (Pedro's), so they go to the field frame and then to
   * the driver frame the drive takes. While aiming the heading lock does the turning, as it does for the driver.
   * Straight to the drive: no stick curve, so Pedro's powers are the motors' powers.
   */
  private void driveAuto(boolean aiming, Aim aim, double lockRateRadPerSec, FieldPose pose, double[] velocity) {
    double[] powers = autoDriver.drive(pose, velocity); // {forward, left, counterclockwise}
    if (powers == null) { // Hold still: no aim turn either
      robot.drive.drive(0, 0, 0, true, pose.headingRad - robot.driverForwardHeadingRad());
      headingHold.clear();
      lastTranslation = new double[] {0, 0};
      Logger.recordOutput("Auto/DrivePowers", new double[] {0, 0, 0});
      return;
    }
    double f = robot.driverForwardHeadingRad();
    double[] translation = robotToDriverFrame(powers[0], powers[1], pose.headingRad, f);
    double magnitude = Math.hypot(translation[0], translation[1]);
    if (aiming && magnitude > DriverConstants.AIM_DRIVE_MAX_POWER) { // Slower while shooting, as for the driver
      translation[0] *= DriverConstants.AIM_DRIVE_MAX_POWER / magnitude;
      translation[1] *= DriverConstants.AIM_DRIVE_MAX_POWER / magnitude;
    }
    double forward = translation[0];
    double right = translation[1];
    double driverHeadingRad = pose.headingRad - f;
    if (aiming && aim != null) {
      robot.drive.driveHeadingLock(forward, right, aim.lockHeadingRad - f, lockRateRadPerSec, driverHeadingRad,
          velocity[2]);
    } else if (!Double.isNaN(autoHeadingRad)) {
      // Pedro's turn fades under the friction near its heading: 14 Nov it stalled 8 degrees off at 0.14 power, where
      // the lock's least power and stuck boost turn the robot
      robot.drive.driveHeadingLock(forward, right, autoHeadingRad - f, autoHeadingRateRadPerSec, driverHeadingRad,
          velocity[2]);
    } else {
      robot.drive.drive(forward, right, -powers[2], true, driverHeadingRad);
    }
    headingHold.clear();
    lastTranslation = new double[] {forward, right};
    Logger.recordOutput("Auto/DrivePowers", powers);
  }

  /**
   * A robot-frame translation {forward, left} as the driver-frame {forward, right} the field-centric drive takes,
   * for a robot at headingRad and a driver whose forward is driverForwardRad.
   */
  static double[] robotToDriverFrame(double forward, double left, double headingRad, double driverForwardRad) {
    double fieldX = forward * Math.cos(headingRad) - left * Math.sin(headingRad);
    double fieldY = forward * Math.sin(headingRad) + left * Math.cos(headingRad);
    return new double[] {fieldX * Math.cos(driverForwardRad) + fieldY * Math.sin(driverForwardRad),
        fieldX * Math.sin(driverForwardRad) - fieldY * Math.cos(driverForwardRad)};
  }
}
