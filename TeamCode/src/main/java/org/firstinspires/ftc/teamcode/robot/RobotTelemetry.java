package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.Units;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.opmodes.Main;
import org.firstinspires.ftc.teamcode.robot.Robot.AimState;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.shooting.feed.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.VisionResult;
import org.psilynx.psikit.core.Logger;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * What the Robot shows: its INIT status, the driver's lines or every subsystem's on the Driver Station, the
 * co-driver's power summary, and the Panels Field and Telemetry tabs. It reads the Robot's state and changes none.
 */
final class RobotTelemetry {
  private static final double PANELS_TELEMETRY_PERIOD_S = 0.1;
  private static final double DS_TELEMETRY_PERIOD_S = 0.1;

  private final Robot robot;
  private final Telemetry telemetry;
  private final java.util.List<String[]> panelsLines = new java.util.ArrayList<>();
  private final AtomicBoolean panelsBusy = new AtomicBoolean();
  private double lastPanelsSendS = Double.NEGATIVE_INFINITY;
  private double lastTelemetryS = Double.NEGATIVE_INFINITY;

  /**
   * Panels writes to every connected browser on the calling thread, and that write blocks for good once a
   * client stops reading (5 Nov: the OpMode thread hung in field.update(), the robot stopped responding
   * and the app restarted from "stuck in stop()"). So the field is sent from this thread instead.
   */
  private final ExecutorService fieldSender = Executors.newSingleThreadExecutor(r -> {
    Thread t = new Thread(r, "PanelsField");
    t.setDaemon(true);
    return t;
  });
  private final AtomicBoolean fieldBusy = new AtomicBoolean();
  private int fieldSkips;

  RobotTelemetry(Robot robot, Telemetry telemetry) {
    this.robot = robot;
    this.telemetry = telemetry;
  }

  /** The Driver Station shows it at 4 Hz at most, and its formatting cost 10-18 ms a loop (7 Nov, 080123). */
  boolean due() {
    return robot.clock.seconds() - lastTelemetryS >= DS_TELEMETRY_PERIOD_S;
  }

  /** Sends this loop's lines when {@link #due}; otherwise drops the OpMode's, so they don't pile up until then. */
  void update(boolean due, Aim aim, FieldPose pose) {
    if (due) {
      lastTelemetryS = robot.clock.seconds();
      addTelemetry(aim, pose);
      telemetry.update();
    } else {
      telemetry.clear();
    }
  }

  /** INIT: the alliance and whether the shooter, hood and cameras are ready. */
  void addInitTelemetry() {
    String allianceWarning = robot.allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    telemetry.addData("Alliance", robot.alliance + "  (X = BLUE, B = RED)");
    telemetry.addData("Shooter stall", robot.shooter.getStallStatus());
    telemetry.addData("Hood", robot.mechanismsWaitForStart ? "homes at START (nothing moves in INIT)"
        : robot.hood.getHomeResult());
    telemetry.addData("Hood encoders", robot.hood.getEncoderFaultText());
    telemetry.addData("Camera L", robot.vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", robot.vision.describe(VisionSubsystem.Side.RIGHT));
    telemetry.update();
  }

  /**
   * The robot, where it points, and the Cell it is aiming at, on the Panels Field tab. Inches, field
   * centre at the origin, x toward the audience Cells. Panels sends it at most every 100 ms. Off with
   * Main.PANELS_ENABLED.
   */
  void drawField(FieldPose pose, Aim aim) {
    if (!Main.PANELS_ENABLED) {
      return;
    }
    Logger.recordOutput("Panels/FieldSkips", fieldSkips);
    // A send still blocked on a client that stopped reading: skip this frame rather than queue behind it
    if (!fieldBusy.compareAndSet(false, true)) {
      fieldSkips++;
      return;
    }
    double x = pose.xM / Units.M_PER_IN;
    double y = pose.yM / Units.M_PER_IN;
    double heading = pose.headingRad;
    String colour = robot.alliance == Alliance.RED ? "red" : "blue";
    boolean aiming = aim != null;
    double cellX = robot.aimSelector.aimCellX / Units.M_PER_IN;
    double cellY = robot.aimSelector.aimCellY / Units.M_PER_IN;
    fieldSender.execute(() -> {
      try {
        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
        if (aiming) {
          field.setStyle("transparent", "yellow", 0.5);
          field.moveCursor(x, y);
          field.line(cellX, cellY);
          field.setStyle("yellow", "black", 0.5);
          field.moveCursor(cellX, cellY);
          field.circle(3);
        }
        field.setStyle("transparent", colour, 1.5); // Robot: 18 in square, drawn as its 9 in circle
        field.moveCursor(x, y);
        field.circle(9);
        field.line(x + 12 * Math.cos(heading), y + 12 * Math.sin(heading)); // Front
        field.update();
      } finally {
        fieldBusy.set(false);
      }
    });
  }

  /**
   * The flywheels, hood and aim on the Panels Telemetry tab, after any lines added with {@link #addPanelsLine}.
   * Sent from the field's thread for the same reason as the field, and skipped while a send is still blocked.
   */
  void sendPanelsTelemetry(Aim aim, FieldPose pose) {
    double nowS = robot.clock.seconds();
    if (!Main.PANELS_ENABLED || nowS - lastPanelsSendS < PANELS_TELEMETRY_PERIOD_S) {
      panelsLines.clear();
      return;
    }
    final java.util.List<String[]> lines = new java.util.ArrayList<>(panelsLines);
    panelsLines.clear();
    lines.add(new String[] {"Flywheel rpm", String.format("%.0f target / %.0f actual", robot.shooter.getTargetRpm(),
        robot.shooter.getRpm())});
    lines.add(new String[] {"Hood launch deg", String.format("%.1f target / %.1f actual  (lift %.1f / %.1f)",
        HoodSubsystem.ticksToAngle(robot.hood.getTarget()), HoodSubsystem.ticksToAngle(robot.hood.getPosition()),
        robot.hood.getTargetLiftDegrees(), robot.hood.getLiftDegrees())});
    lines.add(new String[] {"Aim", aim == null ? robot.aimSelector.aimState + ", Cell not known"
        : String.format("%s  %s  %.2f m to lip  heading err %.1f deg", robot.aimSelector.aimState,
            aim.backward ? "BACKWARD" : "HOODED", aim.shot.distanceToLipM,
            Math.toDegrees(aim.headingErrorRad(pose.headingRad)))});
    lines.add(new String[] {"Pose", String.format("%.2f, %.2f m  %.0f deg%s", pose.xM, pose.yM,
        Math.toDegrees(pose.headingRad), robot.poseFusion.poseValid ? "" : "  (not placed by tags yet)")});
    lines.add(new String[] {"Battery", String.format("%.2f V", robot.batteryMonitor.batteryVolts)});
    if (!panelsBusy.compareAndSet(false, true)) {
      return;
    }
    lastPanelsSendS = nowS;
    fieldSender.execute(() -> {
      try {
        com.bylazar.telemetry.TelemetryManager panels = com.bylazar.telemetry.PanelsTelemetry.INSTANCE.getTelemetry();
        for (String[] line : lines) {
          panels.addData(line[0], line[1]);
        }
        panels.update();
      } catch (RuntimeException e) {
        // Telemetry only: a failed send must not reach the loop
      } finally {
        panelsBusy.set(false);
      }
    });
  }

  /** Shown on the Panels Telemetry tab this loop, above the robot's own lines. */
  void addPanelsLine(String key, String value) { panelsLines.add(new String[] {key, value}); }

  /** For example "✓ 6 tags Δ3cm (0.1s ago) | up AUDIENCE", or the reason the last tags were not used. */
  private String visionLine() {
    String upCell = "up " + robot.visionCorrector.upCell(robot.alliance);
    VisionResult result = robot.poseFusion.lastVision;
    if (result == null) {
      return "no tags yet | " + upCell;
    }
    String text;
    if (result.accepted) {
      text = String.format("✓ %d tags Δ%.0fcm", result.tagsUsed, Math.hypot(result.dxM, result.dyM) * 100);
    } else if (result.reason == VisionResult.Reason.NO_VALID_TAGS) {
      text = "✗ " + result.firstTagReject;
    } else {
      text = "✗ " + result.reason;
    }
    double sinceS = (robot.vision.getCaptureNanos() - robot.poseFusion.lastVisionNanos) * 1e-9;
    return text + String.format(" (%.1fs ago) | ", sinceS) + upCell;
  }

  private void addTelemetry(Aim aim, FieldPose pose) {
    if (robot.isTelemetryDetail()) {
      addDetailTelemetry(aim, pose);
    } else {
      addDriverTelemetry(aim, pose);
    }
  }

  /** The driver's lines: which shot, whether it's ready and what it waits for, and only the problems to act on. */
  private void addDriverTelemetry(Aim aim, FieldPose pose) {
    if (aim == null) {
      telemetry.addData("Shot", Double.isInfinite(robot.aimSelector.cellSeenAgoS)
          ? "no Cell yet: point the cameras at the tags"
          : "Cell lost: point the cameras at the tags");
    } else {
      telemetry.addData("Shot", String.format("%s  %.2f m  %s",
          robot.aimSelector.aimingAtFieldCell ? robot.alliance + " " + robot.aimSelector.aimCell() : "camera's Cell",
          aim.shot.distanceToLipM, aim.backward ? "BACKWARD" : "HOODED"));
      String state;
      if (robot.aimSelector.stopHint != null) {
        state = robot.aimSelector.stopHint;
      } else if (robot.aimSelector.aimState == AimState.LINING_UP) {
        state = String.format("lining up  wheels %.0f/%.0f  hood %.1f/%.1f  turn %.0f deg", robot.shooter.getRpm(),
            robot.shooter.getTargetRpm(), HoodSubsystem.ticksToAngle(robot.hood.getPosition()),
            HoodSubsystem.ticksToAngle(robot.hood.getTarget()), Math.toDegrees(aim.headingErrorRad(pose.headingRad)));
      } else if (robot.aimSelector.aimState == AimState.NO_SAFE_SHOT) {
        state = "no safe shot: " + aim.shot.reason;
      } else {
        state = robot.aimSelector.aimState == AimState.OFF ? "RT to aim" : robot.aimSelector.aimState.toString();
      }
      telemetry.addData("Aim", state + (robot.rpmTrim != 0 ? String.format("  (trim %+.0f)", robot.rpmTrim) : ""));
    }
    if (robot.volleyMode && robot.fireControl.volleyShot) {
      telemetry.addData("Volley", "balls out %d", robot.fireControl.volleyFeedGate.ballsLeft());
    }
    if (!robot.poseFusion.poseValid) {
      telemetry.addLine("! Pose not placed: point the cameras at the tags");
    }
    if (robot.shooter.isStalled()) {
      telemetry.addLine("! Flywheels " + robot.shooter.getStallStatus());
    }
    if (robot.hood.hasEncoderFault()) {
      telemetry.addLine("! Hood " + robot.hood.getEncoderFaultText());
    }
    String deadCameras = robot.vision.deadCameras();
    if (deadCameras != null) {
      telemetry.addLine("! CAM " + deadCameras + " DEAD: fewer tag fixes");
    }
    telemetry.addData("Ramp", !robot.ramp.isPresent() ? "! no " + HardwareNames.RAMP_SERVO + " in the configuration"
        : robot.ramp.isDeployed() ? "DEPLOYED  (D-pad up stows)" : "stowed  (D-pad up deploys)");
  }

  private void addDetailTelemetry(Aim aim, FieldPose pose) {
    telemetry.addData("Alliance", robot.alliance
        + (robot.poseFusion.poseValid ? "  pose on field" : "  pose not placed yet")
        + (robot.poseFusion.headingValid ? ", heading set" : ", heading from start"));
    telemetry.addData("Aim", robot.aimSelector.aimState);
    if (robot.volleyMode) {
      OneBallFeedGate gate = robot.fireControl.volleyFeedGate;
      telemetry.addData("Volley", robot.fireControl.volleyShot
          ? String.format("balls out %d  (fed %d)  feed %.2f  boost %+.0f",
              gate.ballsLeft(), gate.feedStarts(), robot.volleyFeedPower(), robot.volleyBoostRpm())
          : "RB for a standing volley (RT shoots on the move; none without the Cell)");
    }
    if (aim == null) {
      telemetry.addData("Distance", Double.isInfinite(robot.aimSelector.cellSeenAgoS)
          ? "Cell not seen yet: point the cameras at it"
          : String.format("Cell forgotten (seen %.0f s ago): point the cameras at it", robot.aimSelector.cellSeenAgoS));
    } else {
      telemetry.addData("Distance", String.format("%.2f m to lip  %s  (%s)", aim.shot.distanceToLipM,
          aim.backward ? "BACKWARD, hood down" : "FRONT, hooded",
          robot.aimSelector.aimingAtFieldCell
              ? robot.alliance + " " + robot.aimSelector.aimCell()
                  + (AimConstants.AIM_AT_FRONT_CELL ? " (in front)" : " up, from tags")
              : robot.aimSelector.cellSeenAgoS < 0.5 ? "camera sees Cell"
                  : String.format("odometry, seen %.1f s ago", robot.aimSelector.cellSeenAgoS)));
      telemetry.addData("Shot", aim.shot.valid ? "OK" : aim.shot.reason);
      telemetry.addData("Dist", String.format("%.2f m  toward %.2f  sideways %.2f m/s",
          aim.shot.distanceToLipM, aim.shot.speedTowardCellMps, aim.shot.sidewaysSpeedMps));
      telemetry.addData("RPM", String.format("%.0f target / %.0f actual  (trim %+.0f)",
          robot.shooter.getTargetRpm(), robot.shooter.getRpm(), robot.rpmTrim));
      telemetry.addData("Hood", String.format("%.1f target / %.1f actual deg",
          HoodSubsystem.ticksToAngle(robot.hood.getTarget()), HoodSubsystem.ticksToAngle(robot.hood.getPosition())));
      telemetry.addData("Heading err", String.format("%.1f deg (allowed %.1f)",
          Math.toDegrees(aim.headingErrorRad(pose.headingRad)), Math.toDegrees(aim.headingToleranceRad)));
    }
    telemetry.addData("Pose", String.format("%.2f %.2f m  %.0f deg  (driver %.0f)",
        pose.xM, pose.yM, Math.toDegrees(pose.headingRad),
        Math.toDegrees(pose.headingRad - robot.driverForwardHeadingRad())));
    telemetry.addData("Vision", visionLine());
    telemetry.addData("Hood lift (deg)", "target %.1f, actual %.1f", robot.hood.getTargetLiftDegrees(),
        robot.hood.getLiftDegrees());
    telemetry.addData("Shooter", robot.shooter.getShooterState());
    telemetry.addData("Shooter stall", robot.shooter.getStallStatus());
    telemetry.addData("Hood encoders", robot.hood.getEncoderFaultText());
    telemetry.addData("Intake", robot.intake.getIntakeState());
    telemetry.addData("Intake overload", robot.intake.getOverloadStatus());
    telemetry.addData("Indexer", robot.indexer.getIndexerState());
    telemetry.addData("Feeder", robot.feeder.getFeederState());
    telemetry.addData("Beam T/B", robot.intake.isTopBroken() + " / " + robot.intake.isBottomBroken()
        + (robot.intake.isFull() ? "  FULL" : ""));
    telemetry.addData("Camera L", robot.vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", robot.vision.describe(VisionSubsystem.Side.RIGHT));
  }

  /**
   * Power summary for the co-driver, first on the Driver Station (Main adds it before anything else): intake current
   * and how much of its overload budget is used, the flywheels' current, and the battery. The intake is the one that
   * overheats (13 Nov: 5-6 A for most of a match; ~9 A is a stall).
   */
  void addPowerTelemetry() {
    String allianceWarning = robot.allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    double a = robot.intake.getAmps();
    double load = robot.intake.getLoadFraction();
    String intakeFlag = robot.intake.isOverloaded() ? "TRIPPED - LB to restart"
        : a >= 7 || load >= 0.7 ? "!! HOT - let it rest"
        : a >= 5 || load >= 0.35 ? "! warm"
        : "ok";
    telemetry.addData("INTAKE", String.format("%.1f A  load %.0f%%  %s", a, 100 * load, intakeFlag));
    BatteryMonitor battery = robot.batteryMonitor;
    if (!robot.isTelemetryDetail()) { // Battery on one line, its flag only when it needs doing something about
      String batteryFlag = battery.batteryMinVolts < 11.0 ? "  !! LOW - swap"
          : battery.batteryVolts < 12.0 ? "  ! low" : "";
      telemetry.addData("Battery", String.format("%.1f V%s%s", battery.batteryVolts, batteryFlag,
          robot.shooter.isStalled() ? "   FLYWHEELS STALLED" : ""));
      return;
    }
    String shooterFlag = robot.shooter.isStalled() ? "STALLED" : "ok";
    telemetry.addData("FLYWHEELS", String.format("L %.1f  R %.1f A  %s",
        robot.shooter.getLeftAmps(), robot.shooter.getRightAmps(), shooterFlag));
    String batteryFlag = battery.batteryMinVolts < 11.0 ? "!! LOW - swap"
        : battery.batteryVolts < 12.0 ? "! low" : "ok";
    telemetry.addData("BATTERY", String.format("%.2f V  (lowest %.2f)  %s", battery.batteryVolts,
        battery.batteryMinVolts, batteryFlag));
  }

  void stop() {
    fieldSender.shutdownNow();
  }
}
