package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

/**
 * Runs the indexer servos and the intake one at a time, at full power in the feeding direction, and shows what
 * each draws, to find which one isn't turning. Hold A: left indexer servo. Hold B: right indexer servo. Hold X:
 * intake. The Super Servos have no feedback, but each hub measures its servo bus: the left indexer servo is alone
 * on the Expansion Hub's, the right one shares the Control Hub's with the feeder, which is held off here.
 * Melonbotics' figures: 0.16 A free, about 3.9 A stalled.
 */
@TeleOp(name = "Indexer Intake Test", group = "Test")
public class IndexerIntakeTest extends OpMode {
  private static final int CONTROL_HUB = 173;
  private static final int EXPANSION_HUB = 2;
  private static final double READ_PERIOD_S = 0.2; // Hub current reads are slow; not every loop

  private CRServo indexerL;
  private CRServo indexerR;
  private CRServo feeder;
  private DcMotorEx intake;
  private LynxModule controlHub;
  private LynxModule expansionHub;
  private final ElapsedTime readClock = new ElapsedTime();
  private double servoAmpsControl = Double.NaN;
  private double servoAmpsExpansion = Double.NaN;
  private double intakeAmps = Double.NaN;
  private double peakL;
  private double peakR;
  private double peakIntake;

  @Override
  public void init() {
    // Same directions as IndexerSubsystem and IntakeSubsystem, so + is feeding
    indexerL = hardwareMap.get(CRServo.class, "IndexerServoL");
    indexerL.setDirection(DcMotorSimple.Direction.REVERSE);
    indexerR = hardwareMap.get(CRServo.class, "IndexerServoR");
    indexerR.setDirection(DcMotorSimple.Direction.FORWARD);
    feeder = hardwareMap.get(CRServo.class, "FeederServo");
    intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
    intake.setDirection(DcMotorSimple.Direction.FORWARD);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
      if (m.getModuleAddress() == CONTROL_HUB) {
        controlHub = m;
      } else if (m.getModuleAddress() == EXPANSION_HUB) {
        expansionHub = m;
      }
    }
    allOff();
    telemetry.addLine("Hold A: left indexer (Expansion Hub servo 0). Hold B: right indexer (Control Hub servo 0). "
        + "Hold X: intake. All full power, feeding direction.");
  }

  @Override
  public void loop() {
    // The Driver Hub's gamepad 1 and the Panels one (the PC controller bridge), as Robot merges them
    Gamepad pad = Main.PANELS_ENABLED ? PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1)
        : gamepad1;
    boolean a = pad.a;
    boolean b = pad.b;
    boolean x = pad.x;
    indexerL.setPower(a ? 1.0 : 0.0);
    indexerR.setPower(b ? 1.0 : 0.0);
    intake.setPower(x ? 1.0 : 0.0);
    feeder.setPower(0.0);

    if (readClock.seconds() >= READ_PERIOD_S) {
      readClock.reset();
      servoAmpsControl = servoBusAmps(controlHub);
      servoAmpsExpansion = servoBusAmps(expansionHub);
      intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
      if (a && !Double.isNaN(servoAmpsExpansion)) { peakL = Math.max(peakL, servoAmpsExpansion); }
      if (b && !Double.isNaN(servoAmpsControl)) { peakR = Math.max(peakR, servoAmpsControl); }
      if (x) { peakIntake = Math.max(peakIntake, intakeAmps); }
    }

    telemetry.addData("A  left indexer", String.format(Locale.US, "%s  servo bus %.2f A (peak %.2f)  %s",
        a ? "ON " : "off", servoAmpsExpansion, peakL, verdict(a, servoAmpsExpansion)));
    telemetry.addData("B  right indexer", String.format(Locale.US, "%s  servo bus %.2f A (peak %.2f)  %s",
        b ? "ON " : "off", servoAmpsControl, peakR, verdict(b, servoAmpsControl)));
    telemetry.addData("X  intake", String.format(Locale.US, "%s  %.2f A (peak %.2f), encoder %d, %.0f ticks/s",
        x ? "ON " : "off", intakeAmps, peakIntake, intake.getCurrentPosition(), intake.getVelocity()));
    telemetry.addData("Guide", "servo free ~0.2 A, stalled ~3.9 A, ~0 A = no power or not plugged in. "
        + "Intake was ~0.9 A running free, ~5-10 A jammed");
    telemetry.update();
  }

  private static String verdict(boolean on, double amps) {
    if (!on || Double.isNaN(amps)) {
      return "";
    }
    return amps < 0.06 ? "NOTHING DRAWN: no power, unplugged or dead" : amps > 2.5 ? "STALLED or very loaded"
        : "turning";
  }

  /** The hub's servo bus current, amps; NaN if it can't be read. */
  private static double servoBusAmps(LynxModule module) {
    if (module == null) {
      return Double.NaN;
    }
    try {
      LynxGetADCCommand command = new LynxGetADCCommand(module, LynxGetADCCommand.Channel.SERVO_CURRENT,
          LynxGetADCCommand.Mode.ENGINEERING);
      return command.sendReceive().getValue() / 1000.0; // Milliamps
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return Double.NaN;
    } catch (Exception e) {
      return Double.NaN;
    }
  }

  private void allOff() {
    indexerL.setPower(0);
    indexerR.setPower(0);
    feeder.setPower(0);
    intake.setPower(0);
  }

  @Override
  public void stop() {
    allOff();
  }
}
