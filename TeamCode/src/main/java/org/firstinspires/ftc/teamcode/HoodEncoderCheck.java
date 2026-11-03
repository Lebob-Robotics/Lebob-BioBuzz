package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/** Passive check of every encoder port; never drives a motor or resets an encoder. */
@TeleOp(name = "Hood Encoder Check", group = "Tuning")
public class HoodEncoderCheck extends PsiKitIterativeOpMode {
  private static final String[] NAMES = {
      "HoodMotor", "IntakeMotor", "DriveFL", "DriveFR", "DriveBL", "DriveBR", "ShooterL", "ShooterR"};
  private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
  private final int[] baseline = new int[NAMES.length];

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Hood Encoder Check");
  }
  @Override protected void onPsiKitInit() {
    for (int i = 0; i < NAMES.length; i++) {
      motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
      motors[i].setPower(0);
      motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      baseline[i] = motors[i].getCurrentPosition();
    }
  }
  @Override protected void onPsiKitInitLoop() { readEncoders(); }
  @Override protected void onPsiKitStart() {}
  @Override protected void onPsiKitLoop() { readEncoders(); }

  private void readEncoders() {
    telemetry.addLine("Motors off. Mark the pinion and turn it by hand only within free travel.");
    telemetry.addLine("For a 90-degree pinion turn: REV ~2048 counts; 435 RPM motor ~96 counts.");
    for (int i = 0; i < NAMES.length; i++) {
      int ticks = motors[i].getCurrentPosition();
      int delta = ticks - baseline[i];
      Logger.recordOutput("Hood/EncoderCheck/" + NAMES[i] + "/PositionTicks", ticks);
      Logger.recordOutput("Hood/EncoderCheck/" + NAMES[i] + "/DeltaTicks", delta);
      telemetry.addData(NAMES[i], "position %d, change %+d", ticks, delta);
    }
    telemetry.update();
  }

  @Override protected void onPsiKitStop() {
    for (DcMotorEx motor : motors) {
      if (motor != null) { motor.setPower(0); }
    }
  }
}
