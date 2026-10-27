package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Runs one drive motor at a time, and shows the Pinpoint pose, so you can see
 * which wheel each configured name drives and which way the Pinpoint counts.
 * Hold d-pad up for DriveFL, right for DriveFR, down for DriveBR, left for
 * DriveBL. Every motor runs FORWARD (none reversed, unlike in
 * {@link org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem}), so
 * this shows the raw hardware. The Pinpoint is set up as in
 * {@link org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem}; push the
 * robot by hand to see its X, Y and heading change.
 */
@TeleOp(name = "Drive Check", group = "Test")
public class DriveCheck extends OpMode {
  private static final double POWER = 0.25;
  private static final String[] NAMES = {"DriveFL", "DriveFR", "DriveBR", "DriveBL"};
  private static final String[] BUTTONS = {"Up", "Right", "Down", "Left"};

  private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
  private GoBildaPinpointDriver pinpoint;

  @Override
  public void init() {
    for (int i = 0; i < NAMES.length; i++) {
      motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
      motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
      motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    pinpoint.setOffsets(0.0, -105.0, DistanceUnit.MM);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    // Robot must be stationary during this call.
    pinpoint.resetPosAndIMU();
  }

  @Override
  public void loop() {
    boolean[] held = {gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left};
    for (int i = 0; i < motors.length; i++) {
      double p = held[i] ? POWER : 0.0;
      motors[i].setPower(p);
      telemetry.addData(BUTTONS[i] + " " + NAMES[i], "%+.2f  enc %+.0f t/s  (%s)",
          p, motors[i].getVelocity(), motors[i].getConnectionInfo());
    }

    if (gamepad1.a) {
      pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }
    pinpoint.update();
    Pose2D pose = pinpoint.getPosition();
    telemetry.addData("Pinpoint X", "%+.0f mm", pose.getX(DistanceUnit.MM));
    telemetry.addData("Pinpoint Y", "%+.0f mm", pose.getY(DistanceUnit.MM));
    telemetry.addData("Pinpoint heading", "%+.1f deg", pose.getHeading(AngleUnit.DEGREES));
    telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
    telemetry.addLine("A zeroes the Pinpoint");
    telemetry.update();
  }
}
