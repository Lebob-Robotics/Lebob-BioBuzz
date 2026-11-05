package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Flywheel speed-loop tuning: for each {P, lead seconds} in {@link #CONFIGS}, the same steps and ramps of the RPM target
 * that driving toward and away from the Cell asks for. Read afterwards for how fast the wheels follow a
 * falling target (they only coast down under a weak P) and how far they lag a ramp.
 *
 * <p>Only the flywheels run: the indexer and feeder stay off, so no ball is fired. Stops on a stall trip.
 */
@TeleOp(name = "Flywheel Tune", group = "Tuning")
public class FlywheelTune extends PsiKitIterativeOpMode {
  private static final double[][] CONFIGS = {{150, 0.1}};
  private static final double LOW_RPM = 2900;
  private static final double HIGH_RPM = 3500;
  /** {from, to, seconds}: steps, then ramps at 300 and 600 RPM/s. */
  private static final double[][] PROFILE = {
      {LOW_RPM, LOW_RPM, 2.5}, {HIGH_RPM, HIGH_RPM, 2}, {LOW_RPM, LOW_RPM, 2},
      {LOW_RPM, HIGH_RPM, 2}, {HIGH_RPM, HIGH_RPM, 1}, {HIGH_RPM, LOW_RPM, 2}, {LOW_RPM, LOW_RPM, 1},
      {LOW_RPM, HIGH_RPM, 1}, {HIGH_RPM, HIGH_RPM, 1}, {HIGH_RPM, LOW_RPM, 1}, {LOW_RPM, LOW_RPM, 1}};
  private static final long LOOP_PERIOD_MS = 80; // TeleOp's

  private final ElapsedTime clock = new ElapsedTime();
  private final ElapsedTime loopClock = new ElapsedTime();
  private ShooterSubsystem shooter;
  private int config;
  private int seg;
  private double segStartS;
  private double targetRpm;
  private double rate;
  private boolean finished;
  private String finishReason = "";
  private final double savedP = ShooterSubsystem.VELOCITY_P;
  private final double savedLead = ShooterSubsystem.LEAD_S;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Flywheel Tune");
  }
  @Override protected void onPsiKitInit() {
    for (DcMotorEx item : hardwareMap.getAll(DcMotorEx.class)) { item.setPower(0); }
    shooter = new ShooterSubsystem(hardwareMap);
  }
  @Override protected void onPsiKitInitLoop() {
    telemetry.addLine("Flywheel Tune: flywheels only, no balls fed. Keep clear of the shooter.");
    telemetry.update();
  }
  @Override protected void onPsiKitStart() {
    clock.reset();
    loopClock.reset();
    segStartS = 0;
    apply(CONFIGS[0]);
    shooter.setShooterState(ShooterState.AIM);
  }
  @Override protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    if (!finished && shooter.isStalled()) {
      finish("stall");
    }
    if (!finished) {
      double t = nowS - segStartS;
      if (t >= PROFILE[seg][2]) {
        t -= PROFILE[seg][2];
        segStartS += PROFILE[seg][2];
        seg++;
        if (seg >= PROFILE.length) {
          seg = 0;
          config++;
          if (config >= CONFIGS.length) {
            finish("complete");
          } else {
            apply(CONFIGS[config]);
          }
        }
      }
      double[] s = PROFILE[seg];
      rate = (s[1] - s[0]) / s[2];
      targetRpm = s[0] + rate * t;
    }
    if (finished) {
      shooter.setShooterState(ShooterState.STOP);
    } else {
      shooter.setTargetRpm(targetRpm, rate);
    }
    shooter.periodic();
    Logger.recordOutput("FlywheelTune/Config", config);
    Logger.recordOutput("FlywheelTune/Seg", seg);
    Logger.recordOutput("FlywheelTune/P", ShooterSubsystem.VELOCITY_P);
    Logger.recordOutput("FlywheelTune/LeadS", ShooterSubsystem.LEAD_S);
    Logger.recordOutput("FlywheelTune/TargetRpm", targetRpm);
    Logger.recordOutput("FlywheelTune/RateRpmPerSec", rate);
    Logger.recordOutput("FlywheelTune/Seconds", nowS);
    Logger.recordOutput("FlywheelTune/Finished", finished);
    Logger.recordOutput("FlywheelTune/FinishReason", finishReason);
    telemetry.addData("P", ShooterSubsystem.VELOCITY_P);
    telemetry.addData("Target / RPM", "%.0f / %.0f", targetRpm, shooter.getRpm());
    telemetry.addData("Finished", finished ? finishReason : "no");
    telemetry.update();

    long remaining = LOOP_PERIOD_MS - (long) loopClock.milliseconds();
    if (remaining > 0) { try { Thread.sleep(remaining); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } }
    loopClock.reset();
  }
  private static void apply(double[] c) {
    ShooterSubsystem.VELOCITY_P = c[0];
    ShooterSubsystem.LEAD_S = c[1];
  }
  private void finish(String reason) {
    if (!finished) { finished = true; finishReason = reason; }
  }
  @Override protected void onPsiKitStop() {
    if (shooter != null) {
      shooter.setShooterState(ShooterState.STOP);
    }
    ShooterSubsystem.VELOCITY_P = savedP;
    ShooterSubsystem.LEAD_S = savedLead;
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
