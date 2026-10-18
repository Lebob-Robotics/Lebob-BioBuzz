package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodPreset;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.GateResult;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Solution;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.psilynx.psikit.core.Logger;

import java.util.List;

public class Robot {
  private final Telemetry telemetry;
  private final Gamepad driver;

  public final IntakeSubsystem intake;
  public final MecanumDriveSubsystem drive;
  public final OdometrySubsystem odometry;
  public final ShooterSubsystem shooter;
  public final FeederSubsystem feeder;
  public final IndexerSubsystem indexer;
  public final HoodSubsystem hood;
  public final VisionSubsystem vision;

  public static final double triggerLimit = 0.5;

  /** How the shooter is being driven this loop. */
  private enum Mode {
    AIM, MANUAL, IDLE
  }

  /** Picked by the driver during INIT. */
  private Alliance alliance = Alliance.RED;
  // Last valid solved RPM, held while a solution is briefly invalid. NaN = none yet.
  private double lastAimRpm = Double.NaN;

  // Vision pose correction. The history holds the odometry pose of each loop,
  // stamped with that loop's vision capture time, so a frame's pose can be looked up.
  private final ShootOnTheMove.PoseHistory poseHistory = new ShootOnTheMove.PoseHistory();
  private final ShootOnTheMove.VisionCorrector visionCorrector =
      new ShootOnTheMove.VisionCorrector();
  private long lastHistoryNanos = Long.MIN_VALUE;
  // Last update that looked at tags (not just "no new frame"), for the Driver Station.
  private ShootOnTheMove.VisionResult lastVision = null;
  private long lastVisionNanos = 0;

  public Robot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      Gamepad driver) {
    this.telemetry = telemetry;
    this.driver = driver;

    CommandScheduler.getInstance().reset();

    // Construct subsystems.
    intake = new IntakeSubsystem(hardwareMap);
    drive = new MecanumDriveSubsystem(hardwareMap);
    odometry = new OdometrySubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    vision = new VisionSubsystem(hardwareMap);
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    hood.init();
    shooter.setShooterState(ShooterState.IDLE);
  }

  /** Called repeatedly during INIT: the driver picks the alliance. */
  public void initLoop() {
    pollAlliance();
    telemetry.addData("Alliance", alliance + "  (X = BLUE, B = RED)");
    telemetry.update();
  }

  /** Called once when the OpMode starts: applies the field pose. */
  public void start() {
    // Catch a press made after the last init loop.
    pollAlliance();

    // Statics outlive the OpMode, so start untrusted and consume the handoff.
    ShootOnTheMove.poseValid = false;
    ShootOnTheMove.Pose handoff = ShootOnTheMove.handoffPose;
    ShootOnTheMove.handoffPose = null;
    if (handoff != null) {
      odometry.setPose(handoff.x, handoff.y, handoff.heading);
      ShootOnTheMove.poseValid = true;
    } else if (ShootOnTheMove.USE_DEFAULT_START_POSE) {
      double[] start = alliance == Alliance.RED
          ? ShootOnTheMove.DEFAULT_START_POSE_RED : ShootOnTheMove.DEFAULT_START_POSE_BLUE;
      odometry.setPose(start[0], start[1], start[2]);
      ShootOnTheMove.poseValid = true;
    } else {
      // TODO: assumes the robot is placed square to its wall, facing away from it.
      // The driver's A button re-zeroes this at any time.
      odometry.setHeading(ShootOnTheMove.allianceForwardRad(alliance));
    }
    // With no handoff and no default pose, aim stays off until vision gives a bootstrap fix.
    poseHistory.clear();
    visionCorrector.reset();
    lastHistoryNanos = Long.MIN_VALUE;
    lastVision = null;
    Logger.recordOutput("SOTM/UsedHandoff", handoff != null);
  }

  private void pollAlliance() {
    if (driver.x) {
      alliance = Alliance.BLUE;
    } else if (driver.b) {
      alliance = Alliance.RED;
    }
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    // The Pinpoint is already updated for this loop: PsiKit does it before
    // user code runs, so the pose read below is the one that gets logged.
    Pose2D pose = odometry.getPose();
    double heading = pose.getHeading(AngleUnit.RADIANS);
    double posX = pose.getX(DistanceUnit.METER);
    double posY = pose.getY(DistanceUnit.METER);
    double[] vel = odometry.getVelocityMetersPerSec();
    double omega = vel[2];

    // Vision first, so this loop's solve uses the corrected pose. Subsystem
    // periodic() runs inside CommandScheduler.run() at the end of the loop, so the
    // inputs read here are LAST loop's capture. Its time (captureNanos) is also the
    // loop time, so frame ages are measured against it and replay gives the same result.
    List<ShootOnTheMove.TagObservation> observations = vision.takeNewObservations();
    long visionNanos = vision.getCaptureNanos();
    ShootOnTheMove.VisionResult vr = visionCorrector.process(
        observations, poseHistory, new ShootOnTheMove.Pose(posX, posY, heading),
        vel[0], vel[1], omega, ShootOnTheMove.poseValid, visionNanos);
    if (vr.accepted) {
      odometry.setPose(vr.newPose.x, vr.newPose.y, heading);
      poseHistory.shift(vr.gain * vr.dx, vr.gain * vr.dy);
      // The Pinpoint may not report the new pose until its next update.
      posX = vr.newPose.x;
      posY = vr.newPose.y;
    }
    if (vr.reason != ShootOnTheMove.VisionReason.NO_OBSERVATIONS) {
      lastVision = vr;
      lastVisionNanos = visionNanos;
    }
    double frameAgeS = Double.NaN;
    for (ShootOnTheMove.TagObservation o : observations) {
      double age = (visionNanos - o.frameNanos) * 1e-9;
      frameAgeS = Double.isNaN(frameAgeS) ? age : Math.min(frameAgeS, age);
    }
    logVision(vr, frameAgeS, observations.size());

    // Joysticks are relative to the driver's wall, not the field's +x.
    double forwardOffset = ShootOnTheMove.allianceForwardRad(alliance);
    double driverHeading = heading - forwardOffset;

    boolean rightTriggerPressed = driver.right_trigger > triggerLimit;
    boolean leftTriggerPressed = driver.left_trigger > triggerLimit;
    boolean eject = driver.b;
    boolean manualOverride = driver.y;

    // Aim needs a trusted pose and no manual override.
    boolean poseValid = ShootOnTheMove.poseValid;
    Mode mode = Mode.IDLE;
    if (rightTriggerPressed) {
      mode = (poseValid && !manualOverride) ? Mode.AIM : Mode.MANUAL;
    }

    // Solve every loop the pose is trusted, so telemetry and logs show it
    // even when not aiming.
    Solution sol = null;
    GateResult gate = null;
    if (poseValid) {
      sol = ShootOnTheMove.solve(
          new ShootOnTheMove.Pose(posX, posY, heading),
          vel[0], vel[1], omega, ShootOnTheMove.currentGoal(alliance));
      gate = ShootOnTheMove.gate(sol, shooter.getRpm(),
          HoodSubsystem.ticksToAngle(hood.getPosition()), heading, omega);
      logSolution(sol, gate);
    }
    boolean aimValid = mode == Mode.AIM && sol.valid;

    // Driver input: set every subsystem's state for this loop.
    boolean fieldCentric = !driver.left_bumper;
    double forward = -driver.left_stick_y;
    double right = driver.left_stick_x;
    if (aimValid) {
      // Target and heading shifted by the same offset: the error is unchanged
      // and the translation is driver-relative.
      drive.driveHeadingLock(forward, right,
          sol.headingTargetRad - forwardOffset, driverHeading, omega);
    } else {
      drive.drive(forward, right, driver.right_stick_x, fieldCentric, driverHeading);
    }

    if (driver.a) {
      // Driver forward becomes the current heading; keeps x/y.
      odometry.setHeading(forwardOffset);
      // Old samples carry the old heading, so vision cannot use them any more.
      poseHistory.clear();
      heading = forwardOffset;
    }

    Logger.recordOutput("Robot/Driver/Shoot", rightTriggerPressed);
    Logger.recordOutput("Robot/Driver/Intake", leftTriggerPressed || rightTriggerPressed);
    Logger.recordOutput("Robot/Driver/Eject", eject);
    Logger.recordOutput("Robot/Driver/StopShooter", driver.x);
    Logger.recordOutput("Robot/Driver/FieldCentric", fieldCentric);
    Logger.recordOutput("Robot/Driver/ResetHeading", driver.a);
    Logger.recordOutput("Robot/Driver/ManualOverride", manualOverride);
    Logger.recordOutput("Robot/Mode", mode.name());
    Logger.recordOutput("Robot/Alliance", alliance.name());
    Logger.recordOutput("Robot/PoseValid", poseValid);

    ShooterState newShooterState = ShooterState.IDLE;
    if (mode == Mode.AIM) {
      newShooterState = ShooterState.AIM;
    } else if (rightTriggerPressed) {
      newShooterState = ShooterState.SHOOT;
    } else if (eject) {
      newShooterState = ShooterState.EJECT;
    } else if (driver.x) {
      newShooterState = ShooterState.STOP;
    }
    shooter.setShooterState(newShooterState);
    if (mode == Mode.AIM) {
      // Keep the last good target through a brief invalid solution.
      if (sol.valid) {
        lastAimRpm = sol.targetRpm;
      }
      shooter.setTargetRpm(Double.isNaN(lastAimRpm) ? ShootOnTheMove.IDLE_AIM_RPM : lastAimRpm);
    }

    IntakeState newIntakeState = IntakeState.STOP;
    if (eject) {
      newIntakeState = IntakeState.EJECT;
    } else if (leftTriggerPressed || rightTriggerPressed) {
      newIntakeState = IntakeState.INTAKE;
    }
    intake.setIntakeState(newIntakeState);

    FeederState newFeederState = FeederState.STOP;
    if (eject) {
      newFeederState = FeederState.EJECT;
    } else if (leftTriggerPressed || rightTriggerPressed) {
      newFeederState = FeederState.FEED;
    }
    feeder.setFeederState(newFeederState);

    // Intaking loads the tunnel; the indexer holds back so balls cannot creep
    // into the shooter. Only RT feeds, and in AIM each ball waits for the gate.
    IndexerState newIndexerState = IndexerState.STOP;
    if (eject) {
      newIndexerState = IndexerState.EJECT;
    } else if (rightTriggerPressed) {
      newIndexerState = IndexerState.FEED;
    } else if (leftTriggerPressed) {
      newIndexerState = IndexerState.HOLD;
    }
    // Gate first, so entering FEED never writes full power for a closed gate.
    indexer.setFeedAllowed(mode != Mode.AIM || gate.allowed);
    indexer.setIndexerState(newIndexerState);

    if (aimValid) {
      hood.setTargetTicks(HoodSubsystem.angleToTicks(sol.hoodCommandDeg));
    } else if (driver.dpad_up) {
      hood.setPreset(HoodPreset.FAR);
    } else if (driver.dpad_right) {
      hood.setPreset(HoodPreset.MID);
    } else if (driver.dpad_down) {
      hood.setPreset(HoodPreset.CLOSE);
    } else if (driver.dpad_left) {
      hood.setPreset(HoodPreset.STOW);
    }

    // Run after the states above are set, so the hood P loop and every
    // subsystem's logging see this loop's commands rather than last loop's.
    CommandScheduler.getInstance().run();

    // Stamp this loop's pose with the capture time vision just took (same loop, so a
    // few ms from the pose read). Skipped until the first capture, and for repeats.
    long stamp = vision.getCaptureNanos();
    if (stamp > lastHistoryNanos && stamp > 0) {
      poseHistory.add(stamp, new ShootOnTheMove.Pose(posX, posY, heading));
      lastHistoryNanos = stamp;
    }

    addTelemetry(mode, sol, gate, heading);
    telemetry.update();
  }

  private static void logVision(ShootOnTheMove.VisionResult r, double frameAgeS, int newTags) {
    Logger.recordOutput("SOTM/Vision/Accepted", r.accepted);
    Logger.recordOutput("SOTM/Vision/Reason", r.reason.name());
    Logger.recordOutput("SOTM/Vision/TagReject", r.firstTagReject.name());
    Logger.recordOutput("SOTM/Vision/NewTags", newTags);
    Logger.recordOutput("SOTM/Vision/DxM", r.dx);
    Logger.recordOutput("SOTM/Vision/DyM", r.dy);
    Logger.recordOutput("SOTM/Vision/Gain", r.gain);
    Logger.recordOutput("SOTM/Vision/TagsUsed", r.tagsUsed);
    Logger.recordOutput("SOTM/Vision/TagsRejected", r.tagsRejected);
    Logger.recordOutput("SOTM/Vision/SpreadM", r.spread);
    Logger.recordOutput("SOTM/Vision/UsedIds", r.usedIds);
    Logger.recordOutput("SOTM/Vision/UsedDx", r.usedDx);
    Logger.recordOutput("SOTM/Vision/UsedDy", r.usedDy);
    Logger.recordOutput("SOTM/Vision/FrameAgeS", frameAgeS);
    Logger.recordOutput("SOTM/Vision/UpCellRed", ShootOnTheMove.upCellRed.name());
    Logger.recordOutput("SOTM/Vision/UpCellBlue", ShootOnTheMove.upCellBlue.name());
    Logger.recordOutput("SOTM/Vision/RedFlipped", r.redFlipped);
    Logger.recordOutput("SOTM/Vision/BlueFlipped", r.blueFlipped);
  }

  /** Logs every Solution and GateResult field as primitives. */
  private static void logSolution(Solution s, GateResult g) {
    Logger.recordOutput("SOTM/Valid", s.valid);
    Logger.recordOutput("SOTM/Reason", s.reason.name());
    Logger.recordOutput("SOTM/PredictedPose/X", s.predictedPose.x);
    Logger.recordOutput("SOTM/PredictedPose/Y", s.predictedPose.y);
    Logger.recordOutput("SOTM/PredictedPose/Heading", s.predictedPose.heading);
    Logger.recordOutput("SOTM/ShooterX", s.shooterX);
    Logger.recordOutput("SOTM/ShooterY", s.shooterY);
    Logger.recordOutput("SOTM/DistanceM", s.distanceM);
    Logger.recordOutput("SOTM/RadialVelMps", s.radialVelMps);
    Logger.recordOutput("SOTM/TangentialVelMps", s.tangentialVelMps);
    Logger.recordOutput("SOTM/BearingRad", s.bearingRad);
    Logger.recordOutput("SOTM/RawLaunchDeg", s.rawLaunchDeg);
    Logger.recordOutput("SOTM/RawExitSpeedMps", s.rawExitSpeedMps);
    Logger.recordOutput("SOTM/LaunchDeg", s.launchDeg);
    Logger.recordOutput("SOTM/ExitSpeedMps", s.exitSpeedMps);
    Logger.recordOutput("SOTM/HeadingTargetRad", s.headingTargetRad);
    Logger.recordOutput("SOTM/HoodCommandDeg", s.hoodCommandDeg);
    Logger.recordOutput("SOTM/TargetRpm", s.targetRpm);
    Logger.recordOutput("SOTM/TofS", s.tofS);
    Logger.recordOutput("SOTM/ApproachAngleRad", s.approachAngleRad);

    Logger.recordOutput("SOTM/Gate/Valid", g.valid);
    Logger.recordOutput("SOTM/Gate/RpmOk", g.rpmOk);
    Logger.recordOutput("SOTM/Gate/HoodOk", g.hoodOk);
    Logger.recordOutput("SOTM/Gate/HeadingOk", g.headingOk);
    Logger.recordOutput("SOTM/Gate/TurnRateOk", g.turnRateOk);
    Logger.recordOutput("SOTM/Gate/Allowed", g.allowed);
    Logger.recordOutput("SOTM/Gate/HeadingToleranceRad", g.headingToleranceRad);
  }

  private void addTelemetry(Mode mode, Solution sol, GateResult gate, double heading) {
    telemetry.addData("Alliance", alliance + "  poseValid " + ShootOnTheMove.poseValid);
    telemetry.addData("Mode", mode);
    if (sol != null) {
      double headingErrDeg = Math.toDegrees(
          ShootOnTheMove.wrapAngle(heading - sol.headingTargetRad));
      telemetry.addData("Dist", String.format("%.2f m  vr %.2f  vt %.2f",
          sol.distanceM, sol.radialVelMps, sol.tangentialVelMps));
      telemetry.addData("RPM", String.format("%.0f target / %.0f actual",
          sol.targetRpm, shooter.getRpm()));
      telemetry.addData("Hood angle", String.format("%.1f target / %.1f actual deg",
          sol.hoodCommandDeg, HoodSubsystem.ticksToAngle(hood.getPosition())));
      telemetry.addData("Heading err", String.format("%.1f deg", headingErrDeg));
      telemetry.addData("Gate", String.format("%s rpm%s hood%s head%s turn%s",
          gate.allowed ? "GO" : "GATE", mark(gate.rpmOk), mark(gate.hoodOk),
          mark(gate.headingOk), mark(gate.turnRateOk)));
      if (!sol.valid) {
        telemetry.addData("Solution", sol.reason);
      }
    }
    telemetry.addData("Pose", odometry.getPose());
    telemetry.addData("Shooter", shooter.getShooterState()
        + (shooter.isAtSpeed() ? " at speed" : ""));
    telemetry.addData("Feeder", feeder.getFeederState());
    telemetry.addData("Indexer", indexer.getIndexerState()
        + (indexer.isFeedAllowed() ? "" : " (feed blocked)"));
    telemetry.addData("Hood", String.format("%s  %d / %d",
        hood.getPreset(), hood.getPosition(), hood.getTarget()));
    telemetry.addData("Beam T/B", intake.isTopBroken() + " / " + intake.isBottomBroken());
    telemetry.addData("Vision", visionLine());
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
  }

  /** e.g. "✓ 6 tags Δ3cm (0.1s ago) | target AUDIENCE", or the rejection reason. */
  private String visionLine() {
    String target = "target " + ShootOnTheMove.upCell(alliance);
    ShootOnTheMove.VisionResult r = lastVision;
    if (r == null) {
      return "no tags yet | " + target;
    }
    String text;
    if (r.accepted) {
      text = String.format("✓ %d tags Δ%.0fcm", r.tagsUsed, Math.hypot(r.dx, r.dy) * 100);
    } else if (r.reason == ShootOnTheMove.VisionReason.NO_VALID_TAGS) {
      text = "✗ " + r.firstTagReject;
    } else {
      text = "✗ " + r.reason;
    }
    double sinceS = (vision.getCaptureNanos() - lastVisionNanos) * 1e-9;
    return text + String.format(" (%.1fs ago) | ", sinceS) + target;
  }

  private static String mark(boolean ok) {
    return ok ? "✓" : "✗";
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
