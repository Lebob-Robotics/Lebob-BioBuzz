package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Runs one continuous servo at a time, so you can see which roller each
 * configured name actually drives. Hold Y for FeederServo, X for IndexerServoL,
 * B for IndexerServoR; hold the left bumper as well to run it in reverse.
 * Directions here are raw (none reversed), unlike in {@link Robot}.
 */
@TeleOp(name = "Servo Check", group = "Test")
public class ServoCheck extends OpMode {
  private static final String[] NAMES = {"FeederServo", "IndexerServoL", "IndexerServoR"};
  private static final String[] BUTTONS = {"Y", "X", "B"};

  private final CRServo[] servos = new CRServo[NAMES.length];

  @Override
  public void init() {
    for (int i = 0; i < NAMES.length; i++) {
      servos[i] = hardwareMap.get(CRServo.class, NAMES[i]);
    }
  }

  @Override
  public void loop() {
    boolean[] held = {gamepad1.y, gamepad1.x, gamepad1.b};
    double power = gamepad1.left_bumper ? -1.0 : 1.0;
    for (int i = 0; i < servos.length; i++) {
      double p = held[i] ? power : 0.0;
      servos[i].setPower(p);
      telemetry.addData(BUTTONS[i] + " " + NAMES[i],
          "%+.0f  (%s)", p, servos[i].getConnectionInfo());
    }
    telemetry.update();
  }
}
