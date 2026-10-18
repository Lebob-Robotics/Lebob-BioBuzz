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
    }
    // Otherwise aim stays off until vision gives a bootstrap fix.
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
    double[] vel = odometry.getVelocityMetersPerSec();
    double omega = vel[2];

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
          new ShootOnTheMove.Pose(
              pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), heading),
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

    addTelemetry(mode, sol, gate, heading);
    telemetry.update();
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
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
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
