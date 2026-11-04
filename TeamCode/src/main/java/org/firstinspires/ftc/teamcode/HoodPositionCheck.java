package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Bounded hood step test from the lower stop, at the TeleOp's loop period, for each hub gain set in
 * {@link #CONFIGS}: small SOTM-sized corrections (1-2 degrees) and 10-degree moves both ways.
 * Hood must rest on its lower stop at INIT; flywheels stay off.
 *
 * <p>Stops on: an encoder fault, sustained stall current, lift beyond {@link #MAX_LIFT_DEG}, or the
 * sweep ends early when a gain overshoots by more than {@link #MAX_OVERSHOOT_DEG}.
 */
@TeleOp(name = "Hood Position Check", group = "Tuning")
public class HoodPositionCheck extends PsiKitIterativeOpMode {
  /**
   * {position P, velocity P, I, D, F}, gentlest first. The speed loop stays at the SDK default
   * (10, 3, 0, 0), the only one seen well damped on this hood; only the position gain is swept.
   */
  private static final double[][] CONFIGS = {
      {10, 10, 3, 0, 0},
      {20, 10, 3, 0, 0},
      {30, 10, 3, 0, 0}};
  private static final double[] LIFTS_DEG = {3, 4, 6, 16, 6, 5, 3};
  private static final double STEP_SECONDS = 0.6;
  private static final long LOOP_PERIOD_MS = 80; // TeleOp's measured median loop period
  private static final double MAX_LIFT_DEG = 26;
  private static final double MAX_OVERSHOOT_DEG = 1.5;
  private static final double STALL_AMPS = 3.0;
  private static final double STALL_SECONDS = 0.25;

  private final ElapsedTime clock = new ElapsedTime();
  private final ElapsedTime loopClock = new ElapsedTime();
  private HoodSubsystem hood;
  private DcMotorEx motor;
  private boolean finished;
  private String finishReason = "";
  private double highAmpsSince = Double.NaN;
  private int lastStep = -1;
  private double stepStartLift;
  private double stepOvershootDeg;
  /** Gains in force before the sweep, put back on stop so the next OpMode does not inherit a test set. */
  private final double[] savedGains = {HoodSubsystem.POSITION_P, HoodSubsystem.VELOCITY_P,
      HoodSubsystem.VELOCITY_I, HoodSubsystem.VELOCITY_D, HoodSubsystem.VELOCITY_F};

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Hood Position Check");
  }
  @Override protected void onPsiKitInit() {
    for (DcMotorEx item : hardwareMap.getAll(DcMotorEx.class)) { item.setPower(0); }
    hood = new HoodSubsystem(hardwareMap);
    hood.init();
    motor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
  }
  @Override protected void onPsiKitInitLoop() {
    hood.periodic(); // Drives the hood down onto its stop and zeroes it there
    telemetry.addLine("Hood homes to its lower stop, hands clear, flywheels off. Steps 3-4-6-16-6-5-3 deg lift per gain set.");
    telemetry.addData("Lift", "%.2f", hood.getLiftDegrees());
    telemetry.update();
  }
  @Override protected void onPsiKitStart() { clock.reset(); loopClock.reset(); }
  @Override protected void onPsiKitLoop() {
    double seconds = clock.seconds();
    double amps = motor.getCurrent(CurrentUnit.AMPS);
    int step = (int) (seconds / STEP_SECONDS);
    int configIndex = step / LIFTS_DEG.length;
    double lift = hood.getLiftDegrees();

    if (!finished) {
      if (step != lastStep) {
        if (lastStep >= 0 && stepOvershootDeg > MAX_OVERSHOOT_DEG) {
          finish(String.format("overshoot %.2f deg in config %d", stepOvershootDeg, lastStep / LIFTS_DEG.length));
        }
        lastStep = step;
        stepStartLift = lift;
        stepOvershootDeg = 0;
      }
      highAmpsSince = amps >= STALL_AMPS ? (Double.isNaN(highAmpsSince) ? seconds : highAmpsSince) : Double.NaN;
      if (configIndex >= CONFIGS.length) { finish("sweep complete"); }
      else if (hood.hasEncoderFault()) { finish("encoder fault"); }
      else if (!Double.isFinite(amps)) { finish("no current reading"); }
      else if (!Double.isNaN(highAmpsSince) && seconds - highAmpsSince >= STALL_SECONDS) { finish("stall current"); }
      else if (lift > MAX_LIFT_DEG) { finish("lift limit"); }
    }

    if (finished) {
      hood.disable();
    } else {
      double target = LIFTS_DEG[step % LIFTS_DEG.length];
      double[] config = CONFIGS[configIndex];
      HoodSubsystem.POSITION_P = config[0];
      HoodSubsystem.VELOCITY_P = config[1];
      HoodSubsystem.VELOCITY_I = config[2];
      HoodSubsystem.VELOCITY_D = config[3];
      HoodSubsystem.VELOCITY_F = config[4];
      hood.setTargetLiftDegrees(target);
      double direction = Math.signum(target - stepStartLift);
      stepOvershootDeg = Math.max(stepOvershootDeg, direction * (lift - target));
      Logger.recordOutput("Hood/PositionCheck/Config", configIndex);
      Logger.recordOutput("Hood/PositionCheck/Step", step);
      Logger.recordOutput("Hood/PositionCheck/TargetLiftDeg", target);
    }
    hood.periodic();
    Logger.recordOutput("Hood/PositionCheck/Seconds", seconds);
    Logger.recordOutput("Hood/PositionCheck/Amps", amps);
    Logger.recordOutput("Hood/PositionCheck/LiftDeg", lift);
    Logger.recordOutput("Hood/PositionCheck/Finished", finished);
    Logger.recordOutput("Hood/PositionCheck/FinishReason", finishReason);
    telemetry.addData("Lift", "target %.2f, actual %.2f", hood.getTargetLiftDegrees(), lift);
    telemetry.addData("Encoder fault", hood.hasEncoderFault());
    telemetry.addData("Finished", finished ? finishReason : "no");
    telemetry.update();

    // Hold the TeleOp's loop period so the test sees the same retarget rate as a match.
    long remaining = LOOP_PERIOD_MS - (long) loopClock.milliseconds();
    if (remaining > 0) { try { Thread.sleep(remaining); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } }
    loopClock.reset();
  }
  private void finish(String reason) {
    if (!finished) { finished = true; finishReason = reason; }
  }
  @Override protected void onPsiKitStop() {
    if (hood != null) { hood.disable(); }
    HoodSubsystem.POSITION_P = savedGains[0];
    HoodSubsystem.VELOCITY_P = savedGains[1];
    HoodSubsystem.VELOCITY_I = savedGains[2];
    HoodSubsystem.VELOCITY_D = savedGains[3];
    HoodSubsystem.VELOCITY_F = savedGains[4];
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
