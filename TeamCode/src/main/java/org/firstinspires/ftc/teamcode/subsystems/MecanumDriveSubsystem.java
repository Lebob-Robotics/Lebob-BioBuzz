package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;

/**
 * Drivetrain subsystem for a four-motor mecanum base, with optional
 * field-centric driving using the Pinpoint's heading.
 */
public class MecanumDriveSubsystem extends SubsystemBase {
  private final GobildaMotor frontLeft;
  private final GobildaMotor frontRight;
  private final GobildaMotor backLeft;
  private final GobildaMotor backRight;

  public MecanumDriveSubsystem(HardwareMap hardwareMap) {
    // Left motors are flipped so that a positive power on every motor drives
    // straight.
    frontLeft = createDriveMotor(hardwareMap, "DriveFL", true);
    frontRight = createDriveMotor(hardwareMap, "DriveFR", false);
    backLeft = createDriveMotor(hardwareMap, "DriveBL", true);
    backRight = createDriveMotor(hardwareMap, "DriveBR", false);
  }

  private static GobildaMotor createDriveMotor(HardwareMap hardwareMap, String name, boolean reversed) {
    return new GobildaMotor(hardwareMap, name, reversed, DcMotor.RunMode.RUN_USING_ENCODER, false);
  }

  /**
   * Drives the robot given forward/right/rotate joystick components.
   *
   * @param forward        forward power, [-1, 1]
   * @param right          strafe-right power, [-1, 1]
   * @param rotate         clockwise rotation power, [-1, 1]
   * @param fieldCentric   if true, forward/right are relative to the field
   *                       instead of the robot
   * @param headingRadians the robot's current field heading; only used when
   *                       fieldCentric is true
   */
  public void drive(double forward, double right, double rotate, boolean fieldCentric, double headingRadians) {
    if (fieldCentric) {
      double theta = Math.atan2(forward, right);
      double r = Math.hypot(right, forward);

      theta = AngleUnit.normalizeRadians(theta - headingRadians);

      forward = r * Math.sin(theta);
      right = r * Math.cos(theta);
    }

    double frontLeftPower = forward - right + rotate;
    double frontRightPower = forward - right - rotate;
    double backLeftPower = forward + right + rotate;
    double backRightPower = forward + right - rotate;

    // Scale all wheels down together so no power exceeds 1 and the direction
    // of travel is preserved.
    double maxPower = Math.max(1.0,
        Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    frontLeft.setPower(frontLeftPower / maxPower);
    frontRight.setPower(frontRightPower / maxPower);
    backLeft.setPower(backLeftPower / maxPower);
    backRight.setPower(backRightPower / maxPower);
  }

  /** Cuts power to all four wheels. */
  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
