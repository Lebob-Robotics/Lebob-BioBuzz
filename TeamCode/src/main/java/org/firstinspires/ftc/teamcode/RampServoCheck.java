package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;

/**
 * Finds the Pollen ramp's two positions, in the pulse the Hub sends (us). It holds stowed from INIT. D-pad up and down
 * move it 10 us, the bumpers 50 us; A goes to deployed in RampSubsystem, B to stowed. The pulse can go from
 * {@link #LOWER_US} to {@link #UPPER_US}, past the 500-2500 us the REV servo is rated for, to see whether it travels
 * further; where it stops moving is its end.
 */
@TeleOp(name = "Ramp Servo Check", group = "Test")
public class RampServoCheck extends OpMode {
  static final double LOWER_US = 100;
  static final double UPPER_US = 2900;
  private Servo servo;
  private double us = Double.NaN; // NaN without the servo
  private boolean wasUp, wasDown, wasLb, wasRb, wasA, wasB;

  @Override
  public void init() {
    servo = hardwareMap.tryGet(Servo.class, RampSubsystem.SERVO_NAME);
    if (servo instanceof PwmControl) {
      ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(LOWER_US, UPPER_US));
    }
    if (servo != null) {
      send(RampSubsystem.STOWED_US);
    }
  }

  private void send(double pulseUs) {
    us = Math.max(LOWER_US, Math.min(UPPER_US, pulseUs));
    servo.setPosition((us - LOWER_US) / (UPPER_US - LOWER_US));
  }

  @Override
  public void loop() {
    if (servo == null) {
      telemetry.addLine("No " + RampSubsystem.SERVO_NAME + " in the robot configuration");
      telemetry.addLine("Add a Servo named " + RampSubsystem.SERVO_NAME + " on Expansion Hub servo port 1");
      return;
    }
    double next = us;
    if (gamepad1.dpad_up && !wasUp) next = us + 10;
    if (gamepad1.dpad_down && !wasDown) next = us - 10;
    if (gamepad1.right_bumper && !wasRb) next = us + 50;
    if (gamepad1.left_bumper && !wasLb) next = us - 50;
    if (gamepad1.a && !wasA) next = RampSubsystem.DEPLOYED_US;
    if (gamepad1.b && !wasB) next = RampSubsystem.STOWED_US;
    wasUp = gamepad1.dpad_up;
    wasDown = gamepad1.dpad_down;
    wasLb = gamepad1.left_bumper;
    wasRb = gamepad1.right_bumper;
    wasA = gamepad1.a;
    wasB = gamepad1.b;
    if (next != us) {
      send(next);
    }
    telemetry.addData("Pulse", "%.0f us", us);
    telemetry.addData("Stowed / deployed now", "%.0f / %.0f us", RampSubsystem.STOWED_US,
        RampSubsystem.DEPLOYED_US);
    telemetry.addLine("D-pad up/down 10 us, bumpers 50 us, A deployed, B stowed");
  }
}
