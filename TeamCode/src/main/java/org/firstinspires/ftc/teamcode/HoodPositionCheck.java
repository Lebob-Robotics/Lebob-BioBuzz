package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/** Bounded relative-position test at mid-travel; this temporary zero is not a shooting reference. */
@TeleOp(name = "Hood Position Check", group = "Tuning")
public class HoodPositionCheck extends PsiKitIterativeOpMode {
  private final ElapsedTime clock = new ElapsedTime();
  private HoodSubsystem hood;
  private DcMotorEx motor;
  private int motorStart;
  private boolean finished;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Hood Position Check");
  }
  @Override protected void onPsiKitInit() {
    for (DcMotorEx item : hardwareMap.getAll(DcMotorEx.class)) { item.setPower(0); }
    hood = new HoodSubsystem(hardwareMap);
    hood.init();
    motor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
    motorStart = motor.getCurrentPosition();
  }
  @Override protected void onPsiKitInitLoop() {
    hood.periodic();
    telemetry.addLine("Mid-travel relative test only. Hands clear before START. Rehome before shooting.");
    telemetry.update();
  }
  @Override protected void onPsiKitStart() { clock.reset(); }
  @Override protected void onPsiKitLoop() {
    double seconds = clock.seconds();
    double amps = motor.getCurrent(CurrentUnit.AMPS);
    finished |= seconds >= 6.0 || Math.abs(hood.getPosition()) > 1365
        || Math.abs(motor.getCurrentPosition() - motorStart) > 90
        || !Double.isFinite(amps) || amps >= 3.0;
    if (finished) {
      motor.setPower(0);
    } else {
      hood.setTargetLiftDegrees(seconds < 1.5 ? 2 : seconds < 3.0 ? 4 : seconds < 4.5 ? 2 : 0);
      hood.periodic();
    }
    Logger.recordOutput("Hood/PositionCheck/Seconds", seconds);
    Logger.recordOutput("Hood/PositionCheck/Amps", amps);
    Logger.recordOutput("Hood/PositionCheck/Finished", finished);
    telemetry.addData("Lift", "target %.2f, actual %.2f", hood.getTargetLiftDegrees(), hood.getLiftDegrees());
    telemetry.addData("Encoder fault", hood.hasEncoderFault());
    telemetry.addData("Finished", finished);
    telemetry.update();
  }
  @Override protected void onPsiKitStop() {
    if (motor != null) { motor.setPower(0); }
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
