package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Hood tuning for the fast loop, in one of two modes ({@link #MODE}):
 *
 * <ul>
 *   <li>CHARACTERISE: open-loop power pulses up and down at each of {@link #PULSE_POWERS}, with the
 *       loop moving the hood back between them. The full-rate samples in the log give kS, kV, kA, kG.
 *   <li>SWEEP: for each set in {@link #CONFIGS}, the same moves from {@link #LIFTS_DEG}, small SOTM
 *       corrections and 10-25 degree jumps both ways, read afterwards for time to target, overshoot and
 *       settled error.
 *   <li>RANGE: from lift {@link #RANGE_START_DEG}, pushes up at {@link #RANGE_POWER} until the hood stops on the
 *       upper hard stop (or reaches {@link #RANGE_MAX_DEG}), and records that lift: the travel the shots get.
 *   <li>TRACK: the moving targets of driving toward and away from the Cell, {@link #TRACK} at main-loop rate,
 *       first followed in steps (as before rates) and then with their rates. Read afterwards for the lift's
 *       error from the target and how much the hood swings.
 * </ul>
 *
 * <p>Stops on a latched encoder fault, sustained stall current or a lift past {@link #MAX_LIFT_DEG}.
 * Open-loop pulses are cut by the loop thread itself at the edge of their window. Hood homes at INIT;
 * flywheels stay off.
 */
@TeleOp(name = "Hood Tune", group = "Tuning")
public class HoodTune extends PsiKitIterativeOpMode {
  private enum Mode { CHARACTERISE, SWEEP, TRACK, RANGE }
  private static final Mode MODE = Mode.RANGE;

  // ---- RANGE -----------------------------------------------------------------
  private static final double RANGE_START_DEG = 30;
  private static final double RANGE_POWER = 0.3;
  // Past the ~62 that backing-away shots would use; the upper soft limit is widened to it
  private static final double RANGE_MAX_DEG = 64;
  private static final double RANGE_STILL_DEG = 0.05; // Less than this gained in RANGE_STILL_S: on the stop
  private static final double RANGE_STILL_S = 0.4;
  private final double savedMinAngle = HoodSubsystem.MIN_ANGLE_DEG;
  private double rangeTopDeg = Double.NaN;
  private double rangeRefDeg;
  private double rangeRefS;

  // ---- CHARACTERISE --------------------------------------------------------
  private static final double[] PULSE_POWERS = {0.1, 0.15, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0};
  private static final double LOW_DEG = 8;
  private static final double HIGH_DEG = 36;
  private static final double PULSE_MAX_S = 0.6;
  private static final double REPOSITION_S = 1.0;

  // ---- SWEEP -----------------------------------------------------------------
  /**
   * {max speed, accel, decel, kP, kD, kA, kA braking, latency s, take-up per degree, deadband degrees,
   * re-engage degrees}. The first is the current default.
   */
  private static final double[][] CONFIGS = {
      {200, 4000, 3000, 0.2, 0.003, 0.00002, 0.00015, 0.005, 0.05, 0.1, 0.2},
      {200, 4000, 3000, 0.2, 0.003, 0.00002, 0.00015, 0.005, 0.05, 0.1, 0.2}};
  private static final double[] RESET_CONFIG = {60, 600, 600, 0.1, 0.003, 0.00005, 0.00015, 0.005, 0.1, 0.1, 0.2};
  private static final double RESET_SECONDS = 1.5;
  private static final double[] LIFTS_DEG = {10, 30, 31, 33, 32, 40, 25, 27, 40, 39.5, 20, 21, 35, 40, 39, 10};
  private static final double STEP_SECONDS = 0.8;
  private static final double DROP_OVERSHOOT_DEG = 6.0;

  // ---- TRACK -----------------------------------------------------------------
  /** {from lift, to lift, seconds, jitter degrees}: ramps at 4-40 deg/s, holds, and a noisy hold. */
  private static final double[][] TRACK = {
      {15, 15, 1, 0}, {15, 35, 5, 0}, {35, 15, 5, 0}, {15, 35, 2, 0}, {35, 15, 2, 0},
      {15, 35, 1, 0}, {35, 35, 0.6, 0}, {35, 15, 1, 0}, {15, 15, 0.6, 0}, {15, 35, 0.5, 0}, {35, 15, 0.5, 0},
      {15, 15, 0.6, 0}, {25, 25, 3, 0.25}};
  private int pass; // 0: steps, as before rates; 1: with rates
  private int seg;
  private double segStartS = -1;
  private double trackTrueDeg = Double.NaN;
  private double trackRate;
  private final java.util.Random jitter = new java.util.Random(1);
  private final boolean savedTrackRate = HoodSubsystem.FAST_TRACK_RATE;

  /** Main loop period; TeleOp runs at about 80 ms, which the fast loop must not depend on. */
  private static final long LOOP_PERIOD_MS = 80;
  private static final double MAX_LIFT_DEG = 47.5;
  private static final double STALL_AMPS = 6.0;
  private static final double STALL_SECONDS = 0.3;

  private final ElapsedTime clock = new ElapsedTime();
  private final ElapsedTime loopClock = new ElapsedTime();
  private HoodSubsystem hood;
  private DcMotorEx motor;
  private VoltageSensor battery;
  private double lastBatteryS = -1;
  private int config;
  private int step;
  private double stepStartS = -1; // Negative: the step begins on the next loop, from where the hood is then
  private double resetUntilS = Double.NaN;
  private double stepStartLift;
  private int slipsAtConfigStart;
  private int rehomesAtConfigStart;
  private double highAmpsSince = Double.NaN;
  private boolean finished;
  private String finishReason = "";
  private String dropped = "";
  // CHARACTERISE: pulse index, phase 0..5 (to low, up, brake, to high, down, brake), phase start
  private int pulse;
  private int phase;
  private double phaseStartS;
  private final double[] savedGains = saveGains();

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Hood Tune");
  }
  @Override protected void onPsiKitInit() {
    for (DcMotorEx item : hardwareMap.getAll(DcMotorEx.class)) { item.setPower(0); }
    hood = new HoodSubsystem(hardwareMap);
    hood.init();
    motor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
    battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
  }
  @Override protected void onPsiKitInitLoop() {
    hood.periodic(); // Drives the hood down onto its stop and zeroes it there
    telemetry.addLine("Hood Tune (" + MODE + "): hands clear of the hood, flywheels off.");
    telemetry.addData("Home", hood.getHomeResult());
    telemetry.update();
  }
  @Override protected void onPsiKitStart() {
    clock.reset();
    loopClock.reset();
    slipsAtConfigStart = hood.getSlipCount();
    rehomesAtConfigStart = hood.getRehomes();
  }
  @Override protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    double amps = motor.getCurrent(CurrentUnit.AMPS);
    double lift = hood.getLiftDegrees();
    if (battery != null && nowS - lastBatteryS >= 0.5) {
      lastBatteryS = nowS;
      double volts = battery.getVoltage();
      hood.setBatteryVolts(volts);
      Logger.recordOutput("HoodTune/BatteryVolts", volts);
    }

    if (!finished) {
      highAmpsSince = amps >= STALL_AMPS ? (Double.isNaN(highAmpsSince) ? nowS : highAmpsSince) : Double.NaN;
      if (hood.hasEncoderFault() && !hood.isHoming()) { finish("encoder fault"); }
      else if (!Double.isFinite(amps)) { finish("no current reading"); }
      else if (!Double.isNaN(highAmpsSince) && nowS - highAmpsSince >= STALL_SECONDS) { finish("stall current"); }
      else if (lift > (MODE == Mode.RANGE ? RANGE_MAX_DEG + 0.5 : MAX_LIFT_DEG)) { finish("lift limit"); }
    }
    if (!finished) {
      if (MODE == Mode.CHARACTERISE) {
        characterise(nowS, lift);
      } else if (MODE == Mode.TRACK) {
        track(nowS);
      } else if (MODE == Mode.RANGE) {
        range(nowS, lift);
      } else {
        sweep(nowS, lift);
      }
    }
    if (finished) {
      hood.setOpenLoopPower(Double.NaN);
      hood.disable();
      HoodSubsystem.MIN_ANGLE_DEG = savedMinAngle;
    }
    Logger.recordOutput("HoodTune/Range/TopLiftDeg", rangeTopDeg);
    Logger.recordOutput("HoodTune/Range/TopLaunchDeg", HoodSubsystem.ZERO_ANGLE_DEG - rangeTopDeg);
    hood.periodic();
    Logger.recordOutput("HoodTune/Config", config);
    Logger.recordOutput("HoodTune/Step", step);
    Logger.recordOutput("HoodTune/Pulse", pulse);
    Logger.recordOutput("HoodTune/Phase", phase);
    Logger.recordOutput("HoodTune/TargetLiftDeg",
        finished || MODE != Mode.SWEEP || !Double.isNaN(resetUntilS) ? Double.NaN : LIFTS_DEG[step]);
    Logger.recordOutput("HoodTune/Resetting", !Double.isNaN(resetUntilS));
    Logger.recordOutput("HoodTune/Track/Pass", pass);
    Logger.recordOutput("HoodTune/Track/Seg", seg);
    Logger.recordOutput("HoodTune/Track/TrueDeg", trackTrueDeg);
    Logger.recordOutput("HoodTune/Track/RateDegPerSec", trackRate);
    Logger.recordOutput("HoodTune/LiftDeg", lift);
    Logger.recordOutput("HoodTune/Amps", amps);
    Logger.recordOutput("HoodTune/Seconds", nowS);
    Logger.recordOutput("HoodTune/Finished", finished);
    Logger.recordOutput("HoodTune/FinishReason", finishReason);
    Logger.recordOutput("HoodTune/Dropped", dropped);
    telemetry.addData("Mode", MODE);
    telemetry.addData("Config / pulse", "%d / %d, step %d, phase %d", config, pulse, step, phase);
    telemetry.addData("Lift", "%.2f", lift);
    telemetry.addData("Finished", finished ? finishReason : "no");
    telemetry.update();

    long remaining = LOOP_PERIOD_MS - (long) loopClock.milliseconds();
    if (remaining > 0) { try { Thread.sleep(remaining); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } }
    loopClock.reset();
  }

  private void characterise(double nowS, double lift) {
    if (hood.isHoming()) {
      return;
    }
    double p = PULSE_POWERS[pulse];
    double inPhase = nowS - phaseStartS;
    switch (phase) {
      case 0: // Back to the bottom of the window under control
        hood.setOpenLoopPower(Double.NaN);
        applySweepConfig(RESET_CONFIG);
        hood.setTargetLiftDegrees(LOW_DEG);
        if (inPhase >= REPOSITION_S) { nextPhase(nowS); }
        break;
      case 1: // Up: the loop thread cuts it at the top of the window
        hood.setOpenLoopPower(p, LOW_DEG - 3, HIGH_DEG);
        if (lift >= HIGH_DEG || inPhase >= PULSE_MAX_S) { nextPhase(nowS); }
        break;
      case 2:
      case 5:
        hood.setOpenLoopPower(0, 0, MAX_LIFT_DEG);
        if (inPhase >= 0.25) { nextPhase(nowS); }
        break;
      case 3:
        hood.setOpenLoopPower(Double.NaN);
        hood.setTargetLiftDegrees(HIGH_DEG);
        if (inPhase >= REPOSITION_S) { nextPhase(nowS); }
        break;
      case 4: // Down: cut at the bottom of the window
        hood.setOpenLoopPower(-p, LOW_DEG, HIGH_DEG + 3);
        if (lift <= LOW_DEG || inPhase >= PULSE_MAX_S) { nextPhase(nowS); }
        break;
      default:
        break;
    }
  }

  private void range(double nowS, double lift) {
    if (hood.isHoming()) {
      return;
    }
    double inPhase = nowS - phaseStartS;
    switch (phase) {
      case 0: // Under control to the start, with the upper soft limit opened to RANGE_MAX_DEG
        HoodSubsystem.MIN_ANGLE_DEG = HoodSubsystem.ZERO_ANGLE_DEG - RANGE_MAX_DEG - HoodSubsystem.STOP_MARGIN_DEG;
        hood.setTargetLiftDegrees(RANGE_START_DEG);
        if (inPhase >= 2.0) {
          phase = 1;
          phaseStartS = nowS;
          rangeRefDeg = lift;
          rangeRefS = nowS;
        }
        break;
      case 1: // Slowly up until it stops gaining
        hood.setOpenLoopPower(RANGE_POWER, 0, RANGE_MAX_DEG);
        if (lift > rangeRefDeg + RANGE_STILL_DEG) {
          rangeRefDeg = lift;
          rangeRefS = nowS;
        } else if (nowS - rangeRefS >= RANGE_STILL_S || lift >= RANGE_MAX_DEG) {
          rangeTopDeg = lift;
          phase = 2;
          phaseStartS = nowS;
        }
        if (inPhase >= 8) {
          finish("range: never stopped");
        }
        break;
      case 2: // Back down under control, then done
        hood.setOpenLoopPower(Double.NaN);
        HoodSubsystem.MIN_ANGLE_DEG = savedMinAngle;
        hood.setTargetLiftDegrees(20);
        if (inPhase >= 1.5) {
          finish(String.format("range: top stop at lift %.2f (launch %.1f)", rangeTopDeg,
              HoodSubsystem.ZERO_ANGLE_DEG - rangeTopDeg));
        }
        break;
      default:
        break;
    }
  }

  private void track(double nowS) {
    if (hood.isHoming()) {
      segStartS = -1;
      return;
    }
    if (!Double.isNaN(resetUntilS)) {
      HoodSubsystem.FAST_TRACK_RATE = false;
      hood.setTargetLiftDegrees(TRACK[0][0]);
      trackTrueDeg = Double.NaN;
      if (nowS >= resetUntilS) {
        resetUntilS = Double.NaN;
        segStartS = -1;
      }
      return;
    }
    if (segStartS < 0) { segStartS = nowS; }
    double t = nowS - segStartS;
    if (t >= TRACK[seg][2]) {
      t -= TRACK[seg][2];
      segStartS += TRACK[seg][2];
      seg++;
      if (seg >= TRACK.length) {
        seg = 0;
        pass++;
        if (pass >= 2) {
          finish("track complete");
          return;
        }
        resetUntilS = nowS + RESET_SECONDS;
        return;
      }
    }
    double[] s = TRACK[seg];
    double rate = (s[1] - s[0]) / s[2];
    trackTrueDeg = s[0] + rate * t;
    double goal = trackTrueDeg;
    if (s[3] > 0) { // Pose noise: the target jumps about and its rate isn't quite zero
      goal += (jitter.nextDouble() * 2 - 1) * s[3];
      rate += (jitter.nextDouble() * 2 - 1) * 0.5;
    }
    trackRate = rate;
    HoodSubsystem.FAST_TRACK_RATE = pass == 1;
    hood.setTargetLiftDegrees(goal, rate);
  }

  private void nextPhase(double nowS) {
    phaseStartS = nowS;
    phase++;
    if (phase > 5) {
      phase = 0;
      pulse++;
      if (pulse >= PULSE_POWERS.length) {
        pulse = PULSE_POWERS.length - 1;
        finish("characterisation complete");
      }
    }
  }

  private void sweep(double nowS, double lift) {
    if (hood.isHoming()) {
      stepStartS = -1; // Re-homing after a slip: restart the step once it is done
    } else if (!Double.isNaN(resetUntilS)) {
      applySweepConfig(RESET_CONFIG);
      hood.setTargetLiftDegrees(LIFTS_DEG[0]);
      if (nowS >= resetUntilS) {
        resetUntilS = Double.NaN;
        slipsAtConfigStart = hood.getSlipCount();
        rehomesAtConfigStart = hood.getRehomes();
        stepStartS = -1;
      }
    } else {
      if (stepStartS < 0) { beginStep(step, nowS, lift); }
      double target = LIFTS_DEG[step];
      double overshoot = Math.signum(target - stepStartLift) * (lift - target);
      boolean slipped = hood.getSlipCount() > slipsAtConfigStart || hood.getRehomes() > rehomesAtConfigStart;
      if (overshoot > DROP_OVERSHOOT_DEG || slipped) {
        dropped += String.format("config %d step %d: %s; ", config, step,
            slipped ? "slip" : String.format("overshoot %.1f", overshoot));
        nextConfig();
      } else if (nowS - stepStartS >= STEP_SECONDS) {
        if (step + 1 >= LIFTS_DEG.length) {
          nextConfig();
        } else {
          beginStep(step + 1, nowS, lift);
        }
      }
      if (!finished && Double.isNaN(resetUntilS)) {
        if (stepStartS < 0) { beginStep(step, nowS, lift); }
        applySweepConfig(CONFIGS[config]);
        hood.setTargetLiftDegrees(LIFTS_DEG[step]);
      }
    }
  }
  private void beginStep(int newStep, double nowS, double lift) {
    step = newStep;
    stepStartS = nowS;
    stepStartLift = lift;
  }
  private void nextConfig() {
    if (config + 1 >= CONFIGS.length) {
      finish("sweep complete");
      return;
    }
    config++;
    step = 0;
    stepStartS = -1;
    resetUntilS = clock.seconds() + RESET_SECONDS;
  }
  private static void applySweepConfig(double[] c) {
    HoodSubsystem.FAST_MAX_VEL_DEG_S = c[0];
    HoodSubsystem.FAST_ACCEL_DEG_S2 = c[1];
    HoodSubsystem.FAST_DECEL_DEG_S2 = c[2];
    HoodSubsystem.FAST_KP = c[3];
    HoodSubsystem.FAST_KD = c[4];
    HoodSubsystem.FAST_KA = c[5];
    HoodSubsystem.FAST_KA_DECEL = c[6];
    HoodSubsystem.FAST_LATENCY_S = c[7];
    HoodSubsystem.FAST_TAKE_UP_PER_DEG = c[8];
    HoodSubsystem.FAST_DEADBAND_DEG = c[9];
    HoodSubsystem.FAST_REENGAGE_DEG = c[10];
  }
  private static double[] saveGains() {
    return new double[] {HoodSubsystem.FAST_MAX_VEL_DEG_S, HoodSubsystem.FAST_ACCEL_DEG_S2,
        HoodSubsystem.FAST_DECEL_DEG_S2, HoodSubsystem.FAST_KP, HoodSubsystem.FAST_KD,
        HoodSubsystem.FAST_KA, HoodSubsystem.FAST_KA_DECEL, HoodSubsystem.FAST_LATENCY_S,
        HoodSubsystem.FAST_TAKE_UP_PER_DEG, HoodSubsystem.FAST_DEADBAND_DEG, HoodSubsystem.FAST_REENGAGE_DEG};
  }
  private void finish(String reason) {
    if (!finished) { finished = true; finishReason = reason; }
  }
  @Override protected void onPsiKitStop() {
    if (hood != null) {
      hood.setOpenLoopPower(Double.NaN);
      hood.disable();
    }
    applySweepConfig(savedGains);
    HoodSubsystem.FAST_TRACK_RATE = savedTrackRate;
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
