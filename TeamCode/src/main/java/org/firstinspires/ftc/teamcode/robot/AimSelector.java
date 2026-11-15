package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.robot.Robot.AimState;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.shooting.BackwardShotConstants;
import org.firstinspires.ftc.teamcode.shooting.Shot;
import org.firstinspires.ftc.teamcode.shooting.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooting.ShotPickConstants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.vision.CellSighting;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.psilynx.psikit.core.Logger;

/**
 * What the Robot aims at each loop and with which shot: the Cell in front or the tags' up Cell, hooded or backward
 * (picked once a press), and where shoot on the move is up to, with the driver's ready and stop rumbles.
 */
final class AimSelector {
  private static final double TUNING_HEADING_TOLERANCE_DEG = 1.5;
  private static final double PREPOSITION_SETTLE_S = 0.5;
  private static final double STOP_HINT_RUMBLE_AFTER_S = 0.4;
  private static final int STOP_HINT_RUMBLE_MS = 400;

  private final Robot robot;
  private Cell frontCell = null; // Our Cell on the robot's side of the Hive
  AimState aimState = AimState.OFF;
  boolean tuningBackward = false;
  double tuningRpm = 3400;
  double tuningHoodDeg = HoodSubsystem.ZERO_ANGLE_DEG;
  double[] aimPointOverride = null; // Field {x, y} aimed at instead of the Cell; needs the tags' pose
  boolean allianceFromTags = false; // Takes the alliance of the Cell seen while its own is unknown
  boolean hoodedOnly = false;
  boolean backwardOnly = false;
  double cellSeenAgoS = Double.POSITIVE_INFINITY;
  boolean aimingAtFieldCell = false; // At the up Cell's field position, not the tracker's sighted one
  double aimCellX = 0; // The point aimed at this loop, field metres
  double aimCellY = 0;
  /** Shot direction picked when RT was pressed: true backward, false hooded, null not aiming. */
  private Boolean aimBackwardLatched = null;
  private double backwardBlockedSinceS = Double.NaN;
  private boolean prepositionBackward = false;
  private double prepositionSwitchSinceS = Double.NaN;
  String stopHint; // What the driver must change for the shot to fire; null when it's not how they drive
  private double stopHintSinceS = Double.NaN;
  private boolean stopHintRumbled = false;
  private double[] aimVelocity; // Low-passed {x, y} the aim uses; null until the first loop
  private double aimVelocityS;

  AimSelector(Robot robot) {
    this.robot = robot;
  }

  /** The Cell on the robot's side of the Hive, changing only once the robot is FRONT_CELL_SWITCH_M past the middle. */
  static Cell frontCell(Cell current, double xM) {
    double margin = AimConstants.FRONT_CELL_SWITCH_M;
    if (current == null) {
      return xM >= 0 ? Cell.AUDIENCE : Cell.SCORING;
    }
    if (current == Cell.AUDIENCE && xM < -margin) {
      return Cell.SCORING;
    }
    if (current == Cell.SCORING && xM > margin) {
      return Cell.AUDIENCE;
    }
    return current;
  }

  /** The Cell aimed at: the one in front ({@link #frontCell}), or the tags' up Cell with AIM_AT_FRONT_CELL off. */
  Cell aimCell() {
    return AimConstants.AIM_AT_FRONT_CELL && frontCell != null ? frontCell
        : robot.visionCorrector.upCell(robot.alliance);
  }

  /**
   * Null when the Cell hasn't been seen recently. While aiming, the shot direction (hooded or backward)
   * is picked once, as the one ready soonest (ShootOnTheMove.secondsToReady: turn, hood and wheels), and kept
   * until RT is let go: their headings are 180 degrees apart, so switching between them each loop spins the
   * robot back and forth. A backward shot blocked only by the robot's speed still counts, SHOT_PICK_BLOCKED_S
   * later: the robot then lines up and the hood goes down, and it fires once slowed. Picking only a ready
   * backward shot turned the robot 180 degrees to the hooded one whenever the driver was still moving at the
   * press (5 Nov, 092433). SHOT_PICK_QUICKEST false picks by distance instead (BACKWARD_PICK_MAX_DIST_M).
   */
  Aim aimAtCell(FieldPose pose, double[] velocity, long nowNanos, boolean aiming) {
    robot.tracker.recordPose(nowNanos, pose);
    // Every loop, aiming or not, so it has settled by the time RT is pressed. Only the aim uses it
    // Loop clock, not the frame's capture time: that only moves when a camera frame arrives
    double velocityNowS = robot.clock.seconds();
    aimVelocity = ShootOnTheMove.smoothAimVelocity(aimVelocity, velocity, velocityNowS - aimVelocityS,
        AimConstants.AIM_VELOCITY_SMOOTHING_S);
    aimVelocityS = velocityNowS;
    Logger.recordOutput("SOTM/AimVelocity", aimVelocity);
    // Spinning fast, the frame's pose is too uncertain and moved the Cell 0.17 m during one shot (5 Nov)
    boolean steady = Math.abs(velocity[2]) <= VisionConstants.MAX_OMEGA_FOR_VISION_RAD_S;
    for (CellSighting sighting : robot.vision.takeNewCellSightings()) {
      Alliance other = robot.alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
      if (allianceFromTags && other.ownsCluster(sighting.clusterName) && !robot.tracker.knowsCell(nowNanos)
          && !robot.visionCorrector.upCellConfirmed(robot.alliance)) {
        robot.alliance = other;
      }
      if (robot.alliance.ownsCluster(sighting.clusterName)) {
        if (steady) {
          robot.tracker.update(sighting);
        }
      }
    }
    cellSeenAgoS = robot.tracker.secondsSinceSeen(nowNanos);
    // Once the tags have placed the robot and shown which of our Cells is up, aim at that Cell's place on the
    // field. The tracker's Cell, last seen 6-20 s before, had been dragged 0.38 m toward the down Cell by the
    // pose corrections it follows, and the balls went into the down Cell (5 Nov, 081558)
    if (!aiming || frontCell == null) { // Held for the whole shot
      frontCell = frontCell(frontCell, pose.xM);
    }
    aimingAtFieldCell = AimConstants.AIM_AT_FIELD_CELL
        && (robot.poseValid && (AimConstants.AIM_AT_FRONT_CELL || robot.visionCorrector.upCellConfirmed(robot.alliance))
            || robot.poseSeeded);
    double cellX;
    double cellY;
    if (aimPointOverride != null) {
      if (!robot.poseValid) {
        return null;
      }
      cellX = aimPointOverride[0];
      cellY = aimPointOverride[1];
    } else if (aimingAtFieldCell) {
      double[] point = VisionCorrector.upCellAimPoint(robot.alliance, aimCell());
      cellX = point[0];
      cellY = point[1];
    } else if (robot.tracker.knowsCell(nowNanos)) {
      cellX = robot.tracker.getCellX();
      cellY = robot.tracker.getCellY();
    } else {
      return null;
    }
    aimCellX = cellX;
    aimCellY = cellY;
    Aim hooded = ShootOnTheMove.aimHooded(pose, aimVelocity[0], aimVelocity[1], cellX, cellY);
    if (hoodedOnly) {
      aimBackwardLatched = false;
      prepositionBackward = false;
      return hooded;
    }
    Aim backward = ShootOnTheMove.aimBackward(pose, aimVelocity[0], aimVelocity[1], cellX, cellY);
    if (robot.tuningMode && robot.tuningLive) {
      return tuningBackward ? backward : hooded;
    }
    if (robot.tuningMode) {
      // Tight, so a tuning miss is down to the RPM and hood rather than where in the gate the ball left
      return (tuningBackward ? backward : hooded).withShot(tuningRpm, tuningHoodDeg,
          Math.toRadians(TUNING_HEADING_TOLERANCE_DEG));
    }
    boolean pickBackward;
    boolean frontOfCell = BackwardShotConstants.BACKWARD_SHOTS_ON && wedgeForcesBackward(
        ShootOnTheMove.inFrontOfCell(pose, cellX, cellY, ShotPickConstants.BACKWARD_ONLY_WEDGE_DEG),
        backward.shot.distanceToLipM);
    if (!BackwardShotConstants.BACKWARD_SHOTS_ON || backward.shot.reason == Shot.Reason.OUT_OF_RANGE) {
      pickBackward = false;
    } else if (frontOfCell || backwardOnly) {
      pickBackward = true;
    } else if (ShotPickConstants.SHOT_PICK_QUICKEST) {
      // Whichever is ready soonest from the heading, hood and wheels now (8 Nov): no backward arc
      pickBackward = ShootOnTheMove.backwardQuicker(hooded, backward, pose.headingRad,
          HoodSubsystem.ticksToAngle(robot.hood.getPosition()), robot.shooter.getRpm());
    } else {
      pickBackward = hooded.shot.reason == Shot.Reason.OUT_OF_RANGE
          || backward.shot.distanceToLipM <= ShotPickConstants.BACKWARD_PICK_MAX_DIST_M;
    }
    Logger.recordOutput("SOTM/PickBackward", pickBackward);
    if (!aiming) {
      aimBackwardLatched = null;
      // The shot RT would pick, so the hood can get there first. Only switched once the other one has
      // been the pick for PREPOSITION_SETTLE_S, so turning past the switch point doesn't swing the hood
      // through its whole travel each time (the belt skipped doing that, 4 Nov).
      double nowS = nowNanos * 1e-9;
      if (pickBackward == prepositionBackward) {
        prepositionSwitchSinceS = Double.NaN;
      } else if (Double.isNaN(prepositionSwitchSinceS)) {
        prepositionSwitchSinceS = nowS;
      } else if (nowS - prepositionSwitchSinceS >= PREPOSITION_SETTLE_S) {
        prepositionBackward = pickBackward;
        prepositionSwitchSinceS = Double.NaN;
      }
      return prepositionBackward ? backward : hooded;
    }
    if (aimBackwardLatched == null) {
      aimBackwardLatched = pickBackward;
      backwardBlockedSinceS = Double.NaN;
    }
    if (aimBackwardLatched) {
      double nowS = nowNanos * 1e-9;
      if (backward.shot.valid) {
        backwardBlockedSinceS = Double.NaN;
      } else if (Double.isNaN(backwardBlockedSinceS)) {
        backwardBlockedSinceS = nowS;
      } else if (!frontOfCell && nowS - backwardBlockedSinceS >= ShotPickConstants.BACKWARD_GIVE_UP_S
          && hooded.shot.valid) {
        aimBackwardLatched = false; // For the rest of this press, so it doesn't turn back and forth
      }
    }
    return aimBackwardLatched ? backward : hooded;
  }

  /** Also rumbles once each time the shot becomes ready. */
  void updateAimState(boolean aiming, Aim aim, boolean ready) {
    AimState newState;
    if (!aiming) {
      newState = AimState.OFF;
    } else if (aim == null) {
      newState = AimState.CELL_NOT_SEEN;
    } else if (!aim.shot.valid) {
      newState = AimState.NO_SAFE_SHOT;
    } else if (ready) {
      newState = AimState.READY;
    } else {
      newState = AimState.LINING_UP;
    }
    if (newState == AimState.READY && aimState != AimState.READY) {
      robot.physicalDriver.rumble(150);
    }
    aimState = newState;
    stopHint = aiming ? stopHint(aim, newState, robot.closeMovingGateBypassed) : null;
    double nowS = robot.clock.seconds();
    if (stopHint == null) {
      stopHintSinceS = Double.NaN;
    } else if (Double.isNaN(stopHintSinceS)) {
      stopHintSinceS = nowS;
    }
    if (!aiming) {
      stopHintRumbled = false;
    } else if (!stopHintRumbled && !Double.isNaN(stopHintSinceS) && nowS - stopHintSinceS >= STOP_HINT_RUMBLE_AFTER_S) {
      robot.physicalDriver.rumble(STOP_HINT_RUMBLE_MS); // Once a press: the driver is moving in a way that can't shoot
      stopHintRumbled = true;
    }
    Logger.recordOutput("SOTM/StopHint", stopHint == null ? "" : stopHint);
  }

  /** What the driver must change for the shot to fire ({@link ShootOnTheMove#stopHint}), else null. */
  static String stopHint(Aim aim, AimState state, boolean closeMovingBypassed) {
    if (aim == null || (state != AimState.LINING_UP && state != AimState.NO_SAFE_SHOT)) {
      return null;
    }
    return ShootOnTheMove.stopHint(aim.shot, closeMovingBypassed);
  }

  /** In front of the Cell the backward shot is forced, within the range its table was shot at. */
  static boolean wedgeForcesBackward(boolean inFrontOfCell, double backwardLipM) {
    return inFrontOfCell && backwardLipM <= ShotPickConstants.BACKWARD_WEDGE_MAX_LIP_M;
  }
}
