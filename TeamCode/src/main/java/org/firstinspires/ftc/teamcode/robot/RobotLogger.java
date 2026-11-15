package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.vision.VisionResult;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * What the Robot logs for AdvantageScope each loop besides its subsystems: the aim, the pose correction, the volley
 * and the loop's section times. The log tools and AdvantageScope layouts read these keys by name.
 */
final class RobotLogger {
  private final Robot robot;

  RobotLogger(Robot robot) {
    this.robot = robot;
  }


  /** Everything shoot on the move worked out this loop, for AdvantageScope. */
  void logAim(Aim aim) {
    Logger.recordOutput("SOTM/State", robot.aimState.name());
    Logger.recordOutput("SOTM/Alliance", robot.alliance.name());
    Logger.recordOutput("SOTM/KnowsCell", aim != null);
    Logger.recordOutput("SOTM/CellX", robot.aimCellX);
    Logger.recordOutput("SOTM/CellY", robot.aimCellY);
    Logger.recordOutput("SOTM/AimingAtFieldCell", robot.aimingAtFieldCell);
    Logger.recordOutput("SOTM/UpCell", robot.visionCorrector.upCell(robot.alliance).name());
    Logger.recordOutput("SOTM/AimCell", robot.aimCell().name());
    Logger.recordOutput("SOTM/TrackerCellX", robot.tracker.getCellX()); // To compare with the field Cell
    Logger.recordOutput("SOTM/TrackerCellY", robot.tracker.getCellY());
    // For AdvantageScope's field views: a line from the robot to the Cell it is aiming at, none when unknown
    Logger.recordOutput("SOTM/Cell", aim == null ? new Pose2d[0]
        : new Pose2d[] {new Pose2d(robot.aimCellX, robot.aimCellY, new Rotation2d())});
    Logger.recordOutput("SOTM/CellSeenAgoS", robot.cellSeenAgoS);
    Logger.recordOutput("SOTM/RpmTrim", robot.rpmTrim);
    Logger.recordOutput("SOTM/HeadingTrimDeg", robot.headingTrimDeg);
    Logger.recordOutput("SOTM/TableRpmOffset", robot.tableRpmOffset(aim));
    Logger.recordOutput("SOTM/HoodLiftTrimDeg", robot.hoodLiftTrimDeg);
    Logger.recordOutput("Robot/OneBallShot", robot.tuningMode && (robot.driver.right_bumper || robot.scriptedOneBall)
        && robot.driver.right_trigger <= Robot.TRIGGER_PRESSED);
    Logger.recordOutput("Robot/TuningLive", robot.tuningLive);
    Logger.recordOutput("Robot/PositionHold", robot.isHolding());
    Logger.recordOutput("Robot/IntakeOn", robot.ballPath.intakeOn);
    if (aim == null) {
      return;
    }
    Logger.recordOutput("SOTM/Valid", aim.shot.valid);
    Logger.recordOutput("SOTM/Backward", aim.backward);
    Logger.recordOutput("SOTM/Reason", aim.shot.reason.name());
    Logger.recordOutput("SOTM/DistanceToLipM", aim.shot.distanceToLipM);
    Logger.recordOutput("SOTM/SpeedTowardCellMps", aim.shot.speedTowardCellMps);
    Logger.recordOutput("SOTM/SidewaysSpeedMps", aim.shot.sidewaysSpeedMps);
    Logger.recordOutput("SOTM/TargetRpm", aim.shot.targetRpm);
    Logger.recordOutput("SOTM/HoodDeg", aim.shot.hoodDeg);
    Logger.recordOutput("SOTM/TimeOfFlightS", aim.shot.timeOfFlightS);
    Logger.recordOutput("SOTM/TurningRad", aim.shot.turningRad);
    Logger.recordOutput("SOTM/TargetHeadingRad", aim.targetHeadingRad);
    Logger.recordOutput("SOTM/LockHeadingRad", aim.lockHeadingRad);
    Logger.recordOutput("SOTM/HeadingToleranceRad", aim.headingToleranceRad);
  }

  /** Everything the pose correction worked out this loop. */
  void logVision(VisionResult result, double frameAgeS, int newTags) {
    Logger.recordOutput("Vision/Correction/Accepted", result.accepted);
    Logger.recordOutput("Vision/Correction/Reason", result.reason.name());
    Logger.recordOutput("Vision/Correction/TagReject", result.firstTagReject.name());
    Logger.recordOutput("Vision/Correction/NewTags", newTags);
    Logger.recordOutput("Vision/Correction/DxM", result.dxM);
    Logger.recordOutput("Vision/Correction/DyM", result.dyM);
    Logger.recordOutput("Vision/Correction/Gain", result.gain);
    Logger.recordOutput("Vision/Correction/TagsUsed", result.tagsUsed);
    Logger.recordOutput("Vision/Correction/TagsRejected", result.tagsRejected);
    Logger.recordOutput("Vision/Correction/SpreadM", result.spreadM);
    Logger.recordOutput("Vision/Correction/UsedIds", result.usedIds);
    Logger.recordOutput("Vision/Correction/UsedDxM", result.usedDxM);
    Logger.recordOutput("Vision/Correction/UsedDyM", result.usedDyM);
    Logger.recordOutput("Vision/Correction/FrameAgeS", frameAgeS);
    Logger.recordOutput("Vision/Correction/UpCellRed", robot.visionCorrector.upCell(Alliance.RED).name());
    Logger.recordOutput("Vision/Correction/UpCellBlue", robot.visionCorrector.upCell(Alliance.BLUE).name());
    Logger.recordOutput("Vision/Correction/RedFlipped", result.redFlipped);
    Logger.recordOutput("Vision/Correction/BlueFlipped", result.blueFlipped);
    Logger.recordOutput("Vision/Correction/PoseValid", robot.poseValid);
    Logger.recordOutput("Vision/Correction/HeadingValid", robot.headingValid);
    Logger.recordOutput("Vision/Correction/DHeadingDeg", Math.toDegrees(result.dHeadingRad));
    Logger.recordOutput("Vision/Correction/HeadingGain", result.headingGain);
    Logger.recordOutput("Vision/Correction/HeadingPairs", result.headingPairs);
  }

  /** The volley this loop: its gate, the dip each ball left, and the settings in use. */
  void logVolley() {
    Logger.recordOutput("Volley/Shot", robot.volleyShot);
    Logger.recordOutput("Volley/FeedStarts", robot.volleyFeedGate.feedStarts());
    Logger.recordOutput("Volley/BallsLeft", robot.volleyFeedGate.ballsLeft());
    Logger.recordOutput("Volley/LeftAtRpm", robot.volleyFeedGate.peakAtLeaveRpm());
    Logger.recordOutput("Volley/DipDepthRpm", robot.volleyFeedGate.lastDipDepthRpm());
    Logger.recordOutput("Volley/DoubleBall",
        robot.volleyFeedGate.lastDipDepthRpm() > VolleyConstants.VOLLEY_DOUBLE_DIP_RPM);
    Logger.recordOutput("Volley/HoodDeg", robot.volleyHoodDeg);
    Logger.recordOutput("Volley/BoostRpm", robot.volleyBoostRpm());
    Logger.recordOutput("Volley/FeedPower", robot.volleyFeedPower());
    Logger.recordOutput("Volley/IndexerPower", robot.volleyIndexerPower());
    Logger.recordOutput("Volley/FastRecovery", robot.volleyFastRecovery());
  }

  /** System.nanoTime() as one loop reaches each section, logged under Loop/ to find what slows the loop. */
  static final class LoopTimes {
    long start;
    long vision;
    long aim;
    long aimed;
    long shooter;
    long shooterSet;
    long drive;
    long driveSet;
    long feed;
    long feedSet;
    long run;
    long log;
    long end;

    void record() {
      Logger.recordOutput("Loop/StartMs", (vision - start) * 1e-6);
      Logger.recordOutput("Loop/VisionAndControlMs", (run - vision) * 1e-6);
      Logger.recordOutput("Loop/PoseFromVisionMs", (aim - vision) * 1e-6);
      Logger.recordOutput("Loop/AimMs", (aimed - aim) * 1e-6);
      Logger.recordOutput("Loop/ShooterHoodMs", (shooterSet - shooter) * 1e-6);
      Logger.recordOutput("Loop/DriveMs", (driveSet - drive) * 1e-6);
      Logger.recordOutput("Loop/IntakeFeederMs", (feedSet - feed) * 1e-6);
      Logger.recordOutput("Loop/SubsystemsMs", (log - run) * 1e-6);
      Logger.recordOutput("Loop/TelemetryMs", (end - log) * 1e-6);
    }
  }
}
