package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HubMotors;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the shooter hood.
 *
 * <p>The 435 RPM Yellow Jacket output drives the pinion 1:1. Its own encoder remains on
 * Expansion Hub encoder port 0 (HoodMotor). The REV Through Bore on the pinion is connected
 * to Control Hub encoder port 0 (IntakeMotor); the intake is deliberately power-controlled.
 * Hood position comes from the REV's 8192 counts/rev through the 10:1 hood reduction.
 *
 * <p>{@link HoodLoop} runs the position loop on its own thread every few milliseconds, on the REV
 * itself ({@link HoodController}: a motion profile with feedforward and PD). The TeleOp loop runs at
 * about 12 Hz, far too slow to close a loop on a hood that moves 200 degrees a second. Where that
 * thread cannot run (replay) or has failed, the hub's RUN_TO_POSITION on the motor encoder takes
 * over, with the REV target mapped to a motor target through {@link HoodMotorTargeting}.
 *
 * <p>Targets stay {@link #STOP_MARGIN_DEG} inside both stops, so the hub never drives into one at
 * full power. The hood is not driven at all until something sets a target after init(), so INIT
 * never moves it.
 *
 * <p>Gains go to the hub's motor behind PsiKit's wrapper ({@link HubMotors}); the wrapper drops them.
 * POSITION_P, the VELOCITY_* gains and the margins are live in Panels. All gains are written at
 * every INIT: the hub keeps whatever the last OpMode wrote until it is power-cycled.
 */
@Configurable
public class HoodSubsystem extends SubsystemBase {
  // ---- Fast loop (HoodLoop), in lift degrees, seconds and power at 12 V ------
  /** Off: the hub's RUN_TO_POSITION drives the hood instead. */
  public static boolean FAST_LOOP = true;
  public static double FAST_PERIOD_MS = 5;
  public static double FAST_MAX_VEL_DEG_S = 200;
  public static double FAST_ACCEL_DEG_S2 = 4000;
  public static double FAST_DECEL_DEG_S2 = 3000;
  // Fitted to open-loop pulses (Hood Tune CHARACTERISE, 5 Nov): free speed about 250 deg/s at 12 V,
  // 7000+ deg/s^2 at high power, gravity too small to measure.
  public static double FAST_KS = 0.06;
  public static double FAST_KV = 0.0037;
  public static double FAST_KA = 0.00002;
  public static double FAST_KA_DECEL = 0.00015;
  public static double FAST_KG = 0.0;
  public static double FAST_KP = 0.2;
  public static double FAST_KD = 0.003;
  /**
   * Settles (unpowered, the brake holds) inside this, tighter than DEADBAND_TICKS so the hood rests
   * nearer the target than "at target" needs; driven again once the error grows past the re-engage.
   * 0.1 / 0.2 hunted: each 0.2-degree correction's take-up kick overshot by a degree, and the aimed goal's
   * +-0.4 degree noise standing kept starting them; the hood swung 31-34 at 8 Hz (5 Nov, 084004). The ready
   * gate allows 1.
   */
  public static double FAST_DEADBAND_DEG = 0.3;
  public static double FAST_REENGAGE_DEG = 0.5;
  public static double FAST_RESYNC_DEG = 4.0;
  public static double FAST_MAX_POWER = 1.0;
  /** Start of a move: this power until the hood moves, which takes up the belt's slack first. */
  public static double FAST_TAKE_UP_POWER = 0.4;
  public static double FAST_TAKE_UP_RAMP_PER_S = 3.0;
  public static double FAST_STUCK_S = 0.03;
  public static double FAST_TAKE_UP_PER_DEG = 0.05;
  public static double FAST_TAKE_UP_VEL_DEG_S = 15;
  public static double FAST_TAKE_UP_DEG = 0.1;
  public static double FAST_TAKE_UP_MAX_S = 0.12;
  /** Read-to-drive delay the loop predicts across (see HoodController.Gains.latencyS). */
  public static double FAST_LATENCY_S = 0.005;
  /** Off: a moving target is followed in steps, as each new target arrives (the 5 Nov behaviour). */
  public static boolean FAST_TRACK_RATE = true;
  /** Targets moving slower than this count as still, so pose noise doesn't keep the hood driven. */
  public static double FAST_TRACK_MIN_RATE_DEG_S = 1.0;
  /** Power the guard counts as "trying to move" for its stall check: above kS plus small corrections. */
  private static final double FAST_GUARD_POWER = 0.25;
  /** The loop's REV reading and the main loop's, at rest, may differ by this much before it is dropped. */
  private static final double FAST_MISMATCH_DEG = 1.0;

  /** Hub RUN_TO_POSITION gain (fallback when the fast loop cannot run). Higher is faster until it overshoots. */
  public static double POSITION_P = 20.0;
  /**
   * Hub speed loop under RUN_TO_POSITION, in SDK units. These are the SDK's own goBILDA defaults,
   * the only set seen well damped on this hood; a feedforward drives the motor through the belt
   * slack and overshoots.
   */
  public static double VELOCITY_P = 10.0;
  public static double VELOCITY_I = 3.0;
  public static double VELOCITY_D = 0.0;
  public static double VELOCITY_F = 0.0;
  /** Share of full motor speed RUN_TO_POSITION may use. */
  public static double MAX_SPEED = 1.0;
  /** The hub counts a move done within this many motor ticks (one tick is about 0.09 degrees). */
  public static int HUB_TOLERANCE_TICKS = 1;
  /** REV errors smaller than this (ticks) are on target: 45 ticks is 0.2 degrees. */
  public static int DEADBAND_TICKS = 45;
  /** Targets are kept this far inside both stops. */
  public static double STOP_MARGIN_DEG = ShootOnTheMoveConstants.HOOD_STOP_MARGIN_DEG;
  /** Motor speeds below this (ticks/s) count as at rest; the hub reports speed in 20 tick/s steps. */
  public static double AT_REST_TICKS_PER_SEC = 30;
  /** Motor error the guard treats as "trying to move" for its stall check. */
  private static final int GUARD_BUSY_MOTOR_TICKS = 5;
  /** Backstop on the motor target beyond the REV-mapped range: backlash (16) plus drift. */
  private static final int MOTOR_TARGET_SLACK_TICKS = 20;

  // ---- Geometry --------------------------------------------------------
  /** Launch angle at the lower hard stop, where the encoder is zeroed. */
  public static double ZERO_ANGLE_DEG = 99.0;
  /** Launch angle at the top of the hood's travel; sets the upper soft limit. */
  public static double MIN_ANGLE_DEG = ShootOnTheMoveConstants.HOOD_TOP_STOP_DEG;
  /** Encoder ticks per degree of launch angle: 8192 ticks/rev through the 10:1. */
  public static double TICKS_PER_DEG = 8192.0 * 10.0 / 360.0;

  private final DcMotorEx hoodMotor;
  private final DcMotorEx hubMotor; // Same motor, past the logging wrapper, for gains and tolerance
  private final DcMotorEx revEncoderInput;
  private final HoodLoop fastLoop; // Null where it cannot run: the hub's position loop is used
  private boolean drivingFast;
  private int fastMismatchLoops;
  private final HoodEncoderGuard encoderGuard = new HoodEncoderGuard();
  private final HoodMotorTargeting targeting = new HoodMotorTargeting();
  private int hubTolerance = -1;
  private final ElapsedTime clock = new ElapsedTime();
  private int revZero;
  private int motorZero;
  private int homedMotorZero; // motorZero as zeroed on the stop; rebases to the REV move motorZero only
  private boolean zeroed;
  // Homing at INIT: drive down gently until the motor stalls on the lower stop, then zero there.
  public static double HOME_POWER = 0.25;
  public static double HOME_STILL_S = 0.3; // Stalled this long on the stop counts as down
  public static double HOME_TIMEOUT_S = 3.0; // Zeroes where it is after this, in case it never reads still
  // Homing stops once the motor has run this far past the hood's whole travel: the hood is on the stop by
  // then, and a motor still turning is skipping the pinion over the hood's gear. On 5 Nov (093943) it ran
  // 1785 ticks, over 3x the travel, for the full timeout without stalling.
  public static int HOME_OVERTRAVEL_TICKS = 100;
  // After a cut that was not a stall, the zero waits for the motor to stop (or this long): zeroed while it
  // still coasted, the zero sat 820 REV ticks (3.6 degrees) off the stop (093943).
  public static double HOME_SETTLE_MAX_S = 0.5;
  /** Re-homes after a latched belt fault before the hood stays faulted until the next INIT. */
  private static final int MAX_REHOMES = 3;
  /** Each slip or re-home multiplies the next move's speed by this: a loose belt skips under full power. */
  public static double RETRY_SPEED_FACTOR = 0.6;
  public static double MIN_RETRY_SPEED = 0.3;
  private final HoodRecovery recovery = new HoodRecovery(MAX_REHOMES);
  private double appliedPower = Double.NaN;
  private boolean homing;
  private double homeStartS;
  private double homeStillSinceS = Double.NaN;
  private int homeStartMotorTicks;
  private double homeCutS = Double.NaN; // Set once homing has cut power without a stall and waits to zero
  private String homeCutResult;
  private String homeResult = "not started";
  private boolean enabled; // Set by the first target after init()
  private boolean driving; // In RUN_TO_POSITION
  private int lastMotorTarget = Integer.MIN_VALUE;
  private double appliedPositionP = Double.NaN;
  private final double[] appliedVelocityPidf = new double[4];

  /** Hood positions, each given as a launch angle and held in Through Bore ticks. */
  public enum HoodPreset {
    /** As low as the stop margin allows. */
    STOW(Double.NaN),
    CLOSE(75.0), // TODO tune on robot
    MID(65.0), // TODO tune on robot
    FAR(55.0); // TODO tune on robot

    private final double launchAngleDeg;

    HoodPreset(double launchAngleDeg) {
      this.launchAngleDeg = launchAngleDeg;
    }

    public int getTicks() {
      return Double.isNaN(launchAngleDeg) ? 0 : angleToTicks(launchAngleDeg);
    }
  }

  private HoodPreset preset = HoodPreset.STOW;
  // When on, directTicks overrides the preset (used for continuous aiming).
  private boolean directMode = false;
  private int directTicks = 0;
  private double targetLiftRateDegPerSec = 0.0;

  // What the last loop step saw, kept for logging.
  private int lastPosition;
  private int lastError;
  private int lastMotorPosition;
  private double lastMotorVelocity;
  private boolean lastAtTarget;

  public HoodSubsystem(HardwareMap hardwareMap) {
    hoodMotor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
    hubMotor = HubMotors.unwrap(hoodMotor);
    hoodMotor.setDirection(DcMotor.Direction.FORWARD); // Positive raises the hood (verified on robot)
    hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    cut();
    // Read only: never change IntakeMotor's power, mode, direction or encoder zero here.
    revEncoderInput = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
    fastLoop = HoodLoop.create(hubMotor, HubMotors.unwrap(revEncoderInput), TICKS_PER_DEG);
  }

  /** Copies the Panels-live FAST_* values into the loop's gains. */
  static void fillFastGains(HoodController.Gains g) {
    g.maxVelDegPerSec = FAST_MAX_VEL_DEG_S;
    g.accelDegPerSec2 = FAST_ACCEL_DEG_S2;
    g.decelDegPerSec2 = FAST_DECEL_DEG_S2;
    g.kS = FAST_KS;
    g.kV = FAST_KV;
    g.kA = FAST_KA;
    g.kADecel = FAST_KA_DECEL;
    g.kG = FAST_KG;
    g.kP = FAST_KP;
    g.kD = FAST_KD;
    g.deadbandDeg = FAST_DEADBAND_DEG;
    g.reengageDeg = FAST_REENGAGE_DEG;
    g.resyncDeg = FAST_RESYNC_DEG;
    g.maxPower = FAST_MAX_POWER;
    g.takeUpRampPerSec = FAST_TAKE_UP_RAMP_PER_S;
    g.stuckS = FAST_STUCK_S;
    g.takeUpPower = FAST_TAKE_UP_POWER;
    g.takeUpPerDeg = FAST_TAKE_UP_PER_DEG;
    g.takeUpVelDegPerSec = FAST_TAKE_UP_VEL_DEG_S;
    g.takeUpDeg = FAST_TAKE_UP_DEG;
    g.takeUpMaxS = FAST_TAKE_UP_MAX_S;
    g.latencyS = FAST_LATENCY_S;
    g.trackMinRateDegPerSec = FAST_TRACK_MIN_RATE_DEG_S;
  }

  /** Battery voltage, so the fast loop's power means the same on a fresh battery as on a tired one. */
  public void setBatteryVolts(double volts) {
    if (fastLoop != null) {
      fastLoop.setBatteryVolts(volts);
    }
  }

  /**
   * As {@link #setOpenLoopPower(double, double, double)}, cut outside the range targets may take (the stop
   * margins). NaN returns the fast loop to position control.
   */
  public void setOpenLoopPower(double power) {
    setOpenLoopPower(power, minTargetTicks() / TICKS_PER_DEG, maxTargetTicks() / TICKS_PER_DEG);
  }

  /**
   * Drives a fixed power through the fast loop instead of position control (Hood Tune characterisation),
   * cut by the loop itself once the lift leaves [minDeg, maxDeg] in the direction it drives. NaN returns
   * to position control. The hood still needs a target set to be driven at all.
   */
  public void setOpenLoopPower(double power, double minDeg, double maxDeg) {
    if (fastLoop != null) {
      fastLoop.setOpenLoopPower(power, minDeg, maxDeg);
    }
  }

  private boolean useFastLoop() {
    return FAST_LOOP && fastLoop != null && !fastLoop.hasFailed();
  }

  /**
   * Called once from the OpMode's init(). Drives the hood down onto its lower hard stop and zeroes the
   * encoders there (see {@link #periodic()}), so it no longer has to be lowered by hand. Until that is
   * done the hood is not zeroed and won't take targets.
   */
  public void init() {
    initAtRest();
    recovery.reset();
    startHoming();
  }

  private void startHoming() {
    homing = true;
    zeroed = false;
    homeStartS = clock.seconds();
    homeStillSinceS = Double.NaN;
    homeCutS = Double.NaN;
    homeStartMotorTicks = hoodMotor.getCurrentPosition();
    homeResult = "homing";
    hoodMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hoodMotor.setPower(-HOME_POWER); // Positive raises the hood
  }

  /** True while the hood is still driving down to zero itself. */
  public boolean isHoming() {
    return homing;
  }

  /** Short status for telemetry: homing, homed on the stop, or timed out. */
  public String getHomeResult() {
    return homeResult;
  }

  private void updateHoming() {
    double nowS = clock.seconds();
    if (!Double.isNaN(homeCutS)) {
      if (Math.abs(hoodMotor.getVelocity()) < AT_REST_TICKS_PER_SEC || nowS - homeCutS >= HOME_SETTLE_MAX_S) {
        initAtRest();
        homing = false;
        homeResult = homeCutResult;
      }
      return;
    }
    boolean still = nowS - homeStartS > 0.2 // Give it a moment to start moving
        && Math.abs(hoodMotor.getVelocity()) < AT_REST_TICKS_PER_SEC;
    if (!still) {
      homeStillSinceS = Double.NaN;
    } else if (Double.isNaN(homeStillSinceS)) {
      homeStillSinceS = nowS;
    }
    boolean stalled = !Double.isNaN(homeStillSinceS) && nowS - homeStillSinceS >= HOME_STILL_S;
    boolean timedOut = nowS - homeStartS >= HOME_TIMEOUT_S;
    boolean overTravel = Math.abs(hoodMotor.getCurrentPosition() - homeStartMotorTicks)
        > motorCeilingTicks() + HOME_OVERTRAVEL_TICKS;
    if (stalled) {
      initAtRest(); // Cuts power and zeroes both encoders here, on the stop
      homing = false;
      homeResult = "homed on the lower stop";
    } else if (overTravel || timedOut) {
      cut();
      homeCutS = nowS;
      homeCutResult = overTravel
          ? "WARNING: motor ran past the hood's whole travel without stalling. The pinion is skipping on the"
              + " hood's gear at the lower stop: check its mesh. Zeroed on the stop"
          : "timed out, zeroed where it stopped";
    }
  }

  /** Zeroes the encoders where the hood is now, which must be its lower hard stop. */
  private void initAtRest() {
    cut();
    revZero = revEncoderInput.getCurrentPosition();
    motorZero = hoodMotor.getCurrentPosition();
    homedMotorZero = motorZero;
    if (fastLoop != null) {
      fastLoop.setRevZero(revZero);
      fastLoop.setMotorHome(motorZero, motorCeilingTicks());
    }
    encoderGuard.reset();
    targeting.reset();
    clock.reset();
    directMode = false;
    preset = HoodPreset.STOW;
    enabled = false;
    zeroed = true;
    // Log what the hub held before this OpMode wrote anything (the previous OpMode's gains).
    PIDFCoefficients position = hubMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    PIDFCoefficients velocity = hubMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    Logger.recordOutput("Hood/HubAtInit/PositionP", position.p);
    Logger.recordOutput("Hood/HubAtInit/VelocityPIDF",
        new double[] {velocity.p, velocity.i, velocity.d, velocity.f});
    appliedPositionP = Double.NaN; // Force a full write on the first drive
    java.util.Arrays.fill(appliedVelocityPidf, Double.NaN);
    Logger.recordOutput("Hood/GainsReachHub", hubMotor != hoodMotor);
  }

  @Override
  public void periodic() {
    if (fastLoop != null) {
      fastLoop.heartbeat();
    }
    if (homing) {
      updateHoming();
    } else {
      runPositionLoop();
    }
    Logger.recordOutput("Hood/Homing", homing);
    Logger.recordOutput("Hood/HomeResult", homeResult);

    Logger.recordOutput("Hood/Preset", preset);
    Logger.recordOutput("Hood/DirectMode", directMode);
    Logger.recordOutput("Hood/Enabled", enabled);
    Logger.recordOutput("Hood/Driving", driving);
    Logger.recordOutput("Hood/TargetTicks", getTarget());
    Logger.recordOutput("Hood/PositionTicks", lastPosition);
    Logger.recordOutput("Hood/TargetAngleDeg", ticksToAngle(getTarget()));
    Logger.recordOutput("Hood/AngleDeg", ticksToAngle(lastPosition));
    Logger.recordOutput("Hood/ErrorTicks", lastError);
    Logger.recordOutput("Hood/MotorTargetTicks", lastMotorTarget);
    Logger.recordOutput("Hood/MotorTicks", lastMotorPosition);
    Logger.recordOutput("Hood/MotorTicksSinceHome", lastMotorPosition + motorZero - homedMotorZero);
    if (fastLoop != null && zeroed) {
      fastLoop.setMotorCeiling(motorCeilingTicks());
    }
    Logger.recordOutput("Hood/MotorCeilingTicks", motorCeilingTicks());
    Logger.recordOutput("Hood/MotorVelocityTicksPerSec", lastMotorVelocity);
    Logger.recordOutput("Hood/OffsetMotorTicks", targeting.offset());
    Logger.recordOutput("Hood/OffsetUpMotorTicks", targeting.offsetUp());
    Logger.recordOutput("Hood/OffsetDownMotorTicks", targeting.offsetDown());
    Logger.recordOutput("Hood/HubToleranceTicks", hubTolerance);
    Logger.recordOutput("Hood/PositionP", appliedPositionP);
    Logger.recordOutput("Hood/VelocityPIDF", appliedVelocityPidf);
    Logger.recordOutput("Hood/AtTarget", lastAtTarget);
    Logger.recordOutput("Hood/EncoderGuard/Tripped", encoderGuard.isTripped());
    Logger.recordOutput("Hood/EncoderGuard/Slips", encoderGuard.getSlipCount());
    Logger.recordOutput("Hood/EncoderGuard/RevNotFollowing", encoderGuard.isRevNotFollowing());
    Logger.recordOutput("Hood/Rehomes", recovery.rehomes());
    Logger.recordOutput("Hood/Speed", appliedPower);
    Logger.recordOutput("Hood/EncoderGuard/DifferencePinionTurns", Math.abs(
        lastPosition / HoodEncoderGuard.REV_COUNTS_PER_TURN
        - lastMotorPosition / HoodEncoderGuard.MOTOR_COUNTS_PER_TURN));
    Logger.recordOutput("Hood/Fast/Available", fastLoop != null);
    if (fastLoop != null) {
      Logger.recordOutput("Hood/Fast/Active", fastLoop.isActive());
      Logger.recordOutput("Hood/Fast/Failure", fastLoop.failure());
      Logger.recordOutput("Hood/Fast/MotorTicks", fastLoop.motorTicks());
      Logger.recordOutput("Hood/Fast/AtCeiling", fastLoop.isAtCeiling());
      Logger.recordOutput("Hood/Fast/LiftDeg", fastLoop.posDeg());
      Logger.recordOutput("Hood/Fast/VelDegPerSec", fastLoop.velDegPerSec());
      Logger.recordOutput("Hood/Fast/Power", fastLoop.power());
      Logger.recordOutput("Hood/Fast/SetpointDeg", fastLoop.setpointDeg());
      Logger.recordOutput("Hood/Fast/SetpointVelDegPerSec", fastLoop.setpointVelDegPerSec());
      Logger.recordOutput("Hood/Fast/GoalNowDeg", fastLoop.goalNowDeg());
      Logger.recordOutput("Hood/TargetLiftRateDegPerSec", targetLiftRateDegPerSec);
      Logger.recordOutput("Hood/Fast/Settled", fastLoop.isSettled());
      Logger.recordOutput("Hood/Fast/PeriodMsAvg", fastLoop.periodMsAvg());
      Logger.recordOutput("Hood/Fast/PeriodMsMax", fastLoop.periodMsMax());
      double[][] samples = fastLoop.drainSamples(); // Every loop step, for tuning at full rate
      Logger.recordOutput("Hood/Fast/Samples/Seconds", samples[0]);
      Logger.recordOutput("Hood/Fast/Samples/LiftDeg", samples[1]);
      Logger.recordOutput("Hood/Fast/Samples/SetpointDeg", samples[2]);
      Logger.recordOutput("Hood/Fast/Samples/VelDegPerSec", samples[3]);
      Logger.recordOutput("Hood/Fast/Samples/Power", samples[4]);
    }
  }

  /** One loop step. Also stores what it saw so periodic() can log it. */
  private void runPositionLoop() {
    int position = getPosition();
    int motorPosition = hoodMotor.getCurrentPosition() - motorZero;
    double motorVelocity = hoodMotor.getVelocity();
    int target = getTarget();
    lastPosition = position;
    lastMotorPosition = motorPosition;
    lastMotorVelocity = motorVelocity;
    lastError = target - position;
    lastAtTarget = zeroed && !encoderGuard.isTripped() && Math.abs(lastError) < DEADBAND_TICKS;

    if (fastLoop != null && fastLoop.hasRecoverableFailure()) {
      // The fast loop caught the encoders disagreeing; it has already cut power. Same answer as a guard trip.
      lastAtTarget = false;
      cut();
      if (zeroed && recovery.onTrip()) {
        String why = fastLoop.failure();
        fastLoop.clearFailure();
        encoderGuard.reset();
        startHoming();
        homeResult = "re-homing: " + why + " (" + recovery.rehomes() + " of " + MAX_REHOMES + ")";
      }
      return;
    }
    boolean fast = useFastLoop();
    if (drivingFast && !fast) {
      cut(); // The fast loop failed or was switched off: start the hub's loop afresh
    }
    boolean trying = drivingFast
        ? Math.abs(fastLoop.power()) >= FAST_GUARD_POWER
        : driving && Math.abs(lastMotorTarget - motorPosition) > GUARD_BUSY_MOTOR_TICKS;
    boolean atRest = Math.abs(motorVelocity) < AT_REST_TICKS_PER_SEC;
    if (!zeroed || encoderGuard.update(clock.seconds(), position, motorPosition, trying, atRest)) {
      lastAtTarget = false;
      cut(); // A fault must remove drive power; RUN_TO_POSITION would keep pushing.
      if (zeroed && recovery.onTrip()) {
        // The belt kept skipping, or the REV stopped following. Homing finds the lower stop on the motor's
        // own encoder, so it is safe either way; zero there and try again.
        String why = encoderGuard.isRevNotFollowing() ? "REV not following the motor" : "belt slips";
        encoderGuard.reset();
        startHoming();
        homeResult = "re-homing after " + why + " (" + recovery.rehomes() + " of " + MAX_REHOMES + ")";
      }
      return;
    }
    if (lastAtTarget) {
      recovery.onArrived();
    }
    if (encoderGuard.takeRebase()) { // Belt skipped: the REV is the hood, so move the motor's zero to match it
      recovery.onSlip();
      int motorFromRev = (int) Math.round(position / HoodEncoderGuard.REV_COUNTS_PER_TURN
          * HoodEncoderGuard.MOTOR_COUNTS_PER_TURN);
      motorZero += motorPosition - motorFromRev;
      motorPosition = motorFromRev;
      lastMotorPosition = motorPosition;
      lastMotorTarget = Integer.MIN_VALUE; // Resend the target against the new zero
      targeting.reset();
    }
    if (!enabled) {
      return;
    }
    if (target == 0) { // Rest on the lower stop unpowered; holding just above it only stalls the motor
      if (driving) {
        cut();
      }
      return;
    }
    if (fast) {
      driveFast(target, position, atRest);
      return;
    }

    boolean moveDone = driving && atRest
        && Math.abs(lastMotorTarget - motorPosition) <= GUARD_BUSY_MOTOR_TICKS;
    targeting.observe(position, motorPosition, moveDone, target, DEADBAND_TICKS);
    // A wrong offset (say a REV fault the guard has not yet caught) must not aim past either stop.
    int motorTarget = Math.max(
        (int) Math.round(minTargetTicks() * HoodMotorTargeting.MOTOR_PER_REV) - MOTOR_TARGET_SLACK_TICKS,
        Math.min((int) Math.round(maxTargetTicks() * HoodMotorTargeting.MOTOR_PER_REV) + MOTOR_TARGET_SLACK_TICKS,
            targeting.motorTargetFor(target, position, DEADBAND_TICKS, !atRest)));
    // Rebases move motorZero to the REV; the ceiling stays counted from the stop
    motorTarget = Math.min(motorTarget, motorCeilingTicks() - (motorZero - homedMotorZero));
    if (motorTarget != lastMotorTarget || !driving) {
      hoodMotor.setTargetPosition(motorZero + motorTarget); // Must be set before entering RUN_TO_POSITION
      lastMotorTarget = motorTarget;
    }
    applyGains();
    double speed = recovery.speed(MAX_SPEED, RETRY_SPEED_FACTOR, MIN_RETRY_SPEED);
    if (!driving) {
      hubMotor.setTargetPositionTolerance(HUB_TOLERANCE_TICKS);
      hubTolerance = hubMotor.getTargetPositionTolerance(); // Logged: confirms the write reached the hub
      hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      hoodMotor.setPower(speed);
      appliedPower = speed;
      driving = true;
    } else if (speed != appliedPower) {
      hoodMotor.setPower(speed);
      appliedPower = speed;
    }
  }

  private void driveFast(int target, int position, boolean atRest) {
    // Same encoder, read on two paths: a disagreement at rest means the loop's reading is wrong
    if (drivingFast && atRest && Math.abs(fastLoop.posDeg() - position / TICKS_PER_DEG) > FAST_MISMATCH_DEG) {
      if (++fastMismatchLoops >= 3) {
        fastLoop.fail("its REV reading disagreed with the main loop's");
        cut();
        return;
      }
    } else {
      fastMismatchLoops = 0;
    }
    // The rate only applies to the aimed target it came with, and not at either end of the range
    double rate = FAST_TRACK_RATE && directMode && target == directTicks && target != 0 ? targetLiftRateDegPerSec : 0;
    fastLoop.setGoal(target / TICKS_PER_DEG, rate, minTargetTicks() / TICKS_PER_DEG,
        maxTargetTicks() / TICKS_PER_DEG);
    fastLoop.setLimits(0.3, maxTargetTicks() / TICKS_PER_DEG + 0.5);
    fastLoop.setSpeedScale(recovery.speed(1.0, RETRY_SPEED_FACTOR, MIN_RETRY_SPEED));
    if (!drivingFast) {
      fastLoop.activate(position + revZero);
      drivingFast = fastLoop.isActive();
      driving = drivingFast;
    }
  }

  /** Sends gains changed in Panels (or not yet written this OpMode) to the hub. */
  private void applyGains() {
    if (POSITION_P != appliedPositionP) {
      hubMotor.setPositionPIDFCoefficients(POSITION_P);
      appliedPositionP = POSITION_P;
    }
    double[] pidf = {
        VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F};
    if (!java.util.Arrays.equals(pidf, appliedVelocityPidf)) {
      hubMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
          new PIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]));
      System.arraycopy(pidf, 0, appliedVelocityPidf, 0, 4);
    }
  }

  /** Removes drive power until the next target. Open loop, so nothing holds a setpoint. */
  public void disable() {
    enabled = false;
    cut();
  }

  private void cut() {
    if (fastLoop != null) {
      fastLoop.deactivate();
    }
    drivingFast = false;
    hoodMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hoodMotor.setPower(0);
    driving = false;
    appliedPower = Double.NaN;
  }

  public void setPreset(HoodPreset newPreset) {
    preset = newPreset;
    directMode = false;
    enabled = true;
  }

  /**
   * Targets an exact position, overriding the preset until the next
   * {@link #setPreset(HoodPreset)}. Clamped to the soft limits.
   */
  public void setTargetTicks(int ticks) {
    setTargetTicks(ticks, 0.0);
  }

  /**
   * As {@link #setTargetTicks(int)}, for a target moving at liftRateDegPerSec (lift degrees, positive raising
   * the hood): the fast loop follows it between calls instead of in steps. 0 once clamped to a limit.
   */
  public void setTargetTicks(int ticks, double liftRateDegPerSec) {
    directTicks = clampTarget(ticks);
    targetLiftRateDegPerSec = directTicks == ticks ? liftRateDegPerSec : 0.0;
    directMode = true;
    enabled = true;
  }

  /** True while a direct-ticks target is overriding the preset. */
  public boolean isDirectMode() {
    return directMode;
  }

  public HoodPreset getPreset() {
    return preset;
  }

  /**
   * Converts a launch angle to encoder ticks. Ticks count up as the hood
   * raises and the launch angle drops from {@link #ZERO_ANGLE_DEG}.
   */
  public static int angleToTicks(double launchAngleDeg) {
    return (int) Math.round((ZERO_ANGLE_DEG - launchAngleDeg) * TICKS_PER_DEG);
  }

  /** Lowest target, in ticks: the stop margin above the lower stop. */
  public static int minTargetTicks() {
    return (int) Math.round(STOP_MARGIN_DEG * TICKS_PER_DEG);
  }

  /**
   * Motor ticks above the lower stop the hood may never be driven past: the top of travel, plus the belt's
   * backlash and drift. Counted on the motor, so it holds even when the REV stops counting.
   */
  static int motorCeilingTicks() {
    return (int) Math.round(angleToTicks(MIN_ANGLE_DEG) * HoodMotorTargeting.MOTOR_PER_REV) + MOTOR_TARGET_SLACK_TICKS;
  }

  /** Highest target, in ticks: the stop margin below the top of travel. */
  static int maxTargetTicks() {
    return angleToTicks(MIN_ANGLE_DEG + STOP_MARGIN_DEG);
  }

  /** Kept inside the margins, except that a target at or below the lower stop rests on it (0). */
  static int clampTarget(int ticks) {
    if (ticks <= 0) {
      return 0;
    }
    return Math.max(minTargetTicks(), Math.min(maxTargetTicks(), ticks));
  }

  /** Converts encoder ticks to a launch angle in degrees. */
  public static double ticksToAngle(int ticks) {
    return ZERO_ANGLE_DEG - ticks / TICKS_PER_DEG;
  }

  /** Hood position from the REV encoder input, relative to the lower-stop zero. */
  public int getPosition() {
    return revEncoderInput.getCurrentPosition() - revZero;
  }

  /** Physical hood lift above the lower stop, independent of the launch-angle convention. */
  public double getLiftDegrees() {
    return getPosition() / TICKS_PER_DEG;
  }

  public double getTargetLiftDegrees() {
    return getTarget() / TICKS_PER_DEG;
  }

  public void setTargetLiftDegrees(double liftDegrees) {
    setTargetTicks((int) Math.round(liftDegrees * TICKS_PER_DEG));
  }

  /** A lift target moving at liftRateDegPerSec; see {@link #setTargetTicks(int, double)}. */
  public void setTargetLiftDegrees(double liftDegrees, double liftRateDegPerSec) {
    setTargetTicks((int) Math.round(liftDegrees * TICKS_PER_DEG), liftRateDegPerSec);
  }

  public boolean hasEncoderFault() {
    return encoderGuard.isTripped() || fastLoop != null && fastLoop.hasRecoverableFailure();
  }

  /** What the driver should check for a latched fault, or "ok". */
  public String getEncoderFaultText() {
    if (!hasEncoderFault()) {
      return "ok";
    }
    return encoderGuard.isRevNotFollowing() || !encoderGuard.isTripped()
        ? "FAULT: REV encoder not following the motor. Check its cable (Control Hub encoder 0) and that it"
            + " grips the pinion shaft, then re-INIT"
        : "FAULT: check wiring and re-INIT";
  }

  /** Belt slips the encoder guard has seen since INIT. */
  public int getSlipCount() {
    return encoderGuard.getSlipCount();
  }

  /** Re-homes after belt faults since the hood last reached a target. */
  public int getRehomes() {
    return recovery.rehomes();
  }

  /** Target position in ticks: the direct override if set, else the preset's, kept inside the margins. */
  public int getTarget() {
    return clampTarget(directMode ? directTicks : preset.getTicks());
  }

  /** True when the hood is within the deadband of its target. */
  public boolean atTarget() {
    return zeroed && !hasEncoderFault() && Math.abs(getTarget() - getPosition()) < DEADBAND_TICKS;
  }

  /**
   * True when the hood is within toleranceTicks of its target. With the fast loop driving, its reading is
   * used: the main loop's is a loop old, which on a target moving 20 degrees/s is 2 degrees behind already.
   */
  public boolean atTarget(int toleranceTicks) {
    if (!zeroed || hasEncoderFault()) {
      return false;
    }
    double positionTicks = drivingFast && !Double.isNaN(fastLoop.posDeg())
        ? fastLoop.posDeg() * TICKS_PER_DEG : getPosition();
    return Math.abs(getTarget() - positionTicks) <= toleranceTicks;
  }
}
