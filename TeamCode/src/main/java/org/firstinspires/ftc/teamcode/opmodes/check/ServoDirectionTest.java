package org.firstinspires.ftc.teamcode.opmodes.check;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.hardware.OptionalHardware;
import org.firstinspires.ftc.teamcode.opmodes.Main;

/**
 * Finds which way a servo has to turn, any servo in the robot configuration, so a new one needs no code first.
 * D-pad left and right pick the servo. A continuous one runs while Y (+) or A (-) is held, at the power the bumpers
 * set; a positional one steps with the d-pad up and down. Hold X to run the indexer's two stages in their feeding
 * direction at the same time, to see which way the balls have to go. + is the servo's own forward: no reversal.
 */
@TeleOp(name = "Servo Direction Test", group = "Test")
public class ServoDirectionTest extends OpMode {
  static final double POWER_STEP = 0.1;
  static final double POSITION_STEP = 0.05;

  private final List<HardwareDevice> servos = new ArrayList<>();
  private final List<String> names = new ArrayList<>();
  private CRServo indexerL;
  private CRServo indexerR;
  private int picked = 0;
  private double power = 0.5;
  private double position = 0.5;
  private boolean wasLeft, wasRight, wasUp, wasDown, wasLb, wasRb;

  @Override
  public void init() {
    for (CRServo s : hardwareMap.getAll(CRServo.class)) {
      s.setDirection(DcMotorSimple.Direction.FORWARD);
      s.setPower(0);
      add(s);
    }
    for (Servo s : hardwareMap.getAll(Servo.class)) {
      add(s);
    }
    // Feeding directions as IndexerSubsystem's
    indexerL = OptionalHardware.find(hardwareMap, CRServo.class, HardwareNames.INDEXER_SERVO_LEFT);
    indexerR = OptionalHardware.find(hardwareMap, CRServo.class, HardwareNames.INDEXER_SERVO_RIGHT);
    telemetry.addData("Servos found", names.isEmpty() ? "none" : String.join(", ", names));
    telemetry.addLine("Press START. Nothing moves until a button is held.");
  }

  private void add(HardwareDevice device) {
    Set<String> found = hardwareMap.getNamesOf(device);
    servos.add(device);
    names.add(found.isEmpty() ? "?" : found.iterator().next());
  }

  @Override
  public void loop() {
    // The Driver Hub's gamepad 1 and the Panels one (the PC controller bridge), as Robot merges them
    Gamepad pad = Main.PANELS_ENABLED ? PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1)
        : gamepad1;
    if (servos.isEmpty()) {
      telemetry.addLine("No servos in the robot configuration");
      telemetry.update();
      return;
    }
    int was = picked;
    if (pad.dpad_right && !wasRight) picked = (picked + 1) % servos.size();
    if (pad.dpad_left && !wasLeft) picked = (picked + servos.size() - 1) % servos.size();
    if (picked != was) {
      allStopped();
    }
    if (pad.right_bumper && !wasRb) power = Math.min(1.0, power + POWER_STEP);
    if (pad.left_bumper && !wasLb) power = Math.max(POWER_STEP, power - POWER_STEP);

    HardwareDevice device = servos.get(picked);
    String name = names.get(picked);
    String doing;
    if (device instanceof CRServo) {
      double p = pad.y ? power : pad.a ? -power : 0.0;
      ((CRServo) device).setPower(p);
      doing = p > 0 ? "+ (forward)" : p < 0 ? "- (reverse)" : "stopped";
      telemetry.addData("Running", "%s at %.1f", doing, Math.abs(p));
    } else {
      Servo s = (Servo) device;
      if (pad.dpad_up && !wasUp) position = Math.min(1.0, position + POSITION_STEP);
      if (pad.dpad_down && !wasDown) position = Math.max(0.0, position - POSITION_STEP);
      s.setPosition(position);
      telemetry.addData("Position", "%.2f (0 to 1)", position);
    }
    boolean flow = pad.x;
    boolean pickedIsIndexer = name.equals(HardwareNames.INDEXER_SERVO_LEFT)
        || name.equals(HardwareNames.INDEXER_SERVO_RIGHT);
    if (!pickedIsIndexer) {
      if (indexerL != null) indexerL.setPower(flow ? -power : 0.0); // IndexerSubsystem reverses the left stage
      if (indexerR != null) indexerR.setPower(flow ? power : 0.0);
    }
    wasLeft = pad.dpad_left;
    wasRight = pad.dpad_right;
    wasUp = pad.dpad_up;
    wasDown = pad.dpad_down;
    wasLb = pad.left_bumper;
    wasRb = pad.right_bumper;

    telemetry.addData("Servo", "%d of %d: %s (%s)", picked + 1, servos.size(), name,
        device instanceof CRServo ? "continuous" : "positional");
    telemetry.addData("Port", device.getConnectionInfo());
    telemetry.addData("Power", "%.1f", power);
    if (flow && !pickedIsIndexer) {
      telemetry.addLine("Indexer stages feeding");
    }
    telemetry.addLine(String.format(Locale.US, "D-pad left/right: servo. Hold Y +, A -. Bumpers: power. "
        + "D-pad up/down: positional servo. Hold X: indexer feeding."));
    telemetry.addLine("If + moves balls toward the shooter, it needs no reversal (reversed = false)");
    telemetry.update();
  }

  private void allStopped() {
    for (HardwareDevice d : servos) {
      if (d instanceof CRServo) {
        ((CRServo) d).setPower(0);
      }
    }
  }

  @Override
  public void stop() {
    allStopped();
  }
}
