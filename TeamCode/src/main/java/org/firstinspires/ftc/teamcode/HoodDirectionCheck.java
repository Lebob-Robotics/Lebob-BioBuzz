package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.HoodDirectionPulse;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/** Start only with the hood clear of both stops: one bounded pulse each way; flywheels stay off. */
@TeleOp(name = "Hood Direction Check", group = "Tuning")
public class HoodDirectionCheck extends PsiKitIterativeOpMode {
  private final ElapsedTime clock = new ElapsedTime();
  private final HoodDirectionPulse pulse = new HoodDirectionPulse();
  private DcMotorEx motor;
  private DcMotorEx rev;
  private boolean started;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Hood Direction Check");
  }
  @Override protected void onPsiKitInit() {
    for (DcMotorEx item : hardwareMap.getAll(DcMotorEx.class)) { item.setPower(0); }
    motor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
    rev = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setDirection(DcMotorSimple.Direction.FORWARD);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setPower(0);
  }
  @Override protected void onPsiKitInitLoop() { sample(false); }
  @Override protected void onPsiKitStart() { clock.reset(); started = true; }
  @Override protected void onPsiKitLoop() { sample(started); }

  private void sample(boolean running) {
    int revTicks = rev.getCurrentPosition();
    int motorTicks = motor.getCurrentPosition();
    double amps = motor.getCurrent(CurrentUnit.AMPS);
    double seconds = running ? clock.seconds() : -1;
    double power = running ? pulse.update(seconds, revTicks, motorTicks, amps) : 0;
    motor.setPower(power);
    Logger.recordOutput("Hood/DirectionCheck/Seconds", seconds);
    Logger.recordOutput("Hood/DirectionCheck/Power", power);
    Logger.recordOutput("Hood/DirectionCheck/RevTicks", revTicks);
    Logger.recordOutput("Hood/DirectionCheck/MotorTicks", motorTicks);
    Logger.recordOutput("Hood/DirectionCheck/Amps", amps);
    telemetry.addLine(!running ? "Hood must be clear of both stops; hands clear before START."
        : seconds > 0.75 ? "Test finished. Hood power is zero. STOP now."
        : "Brief direction test running; other motors remain off.");
    telemetry.addData("Power / current", "%.2f / %.2f A", power, amps);
    telemetry.addData("REV / motor", "%d / %d", revTicks, motorTicks);
    telemetry.update();
  }
  @Override protected void onPsiKitStop() {
    if (motor != null) { motor.setPower(0); }
  }
}
