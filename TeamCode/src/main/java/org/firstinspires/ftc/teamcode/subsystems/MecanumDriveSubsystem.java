package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.GobildaMotor;
import org.psilynx.psikit.core.Logger;

import java.util.Arrays;

/**
 * Drivetrain subsystem for a four-motor mecanum base, with optional
 * field-centric driving using the Pinpoint's heading.
 */
public class MecanumDriveSubsystem extends SubsystemBase {
  private final GobildaMotor frontLeft;
  private final GobildaMotor frontRight;
  private final GobildaMotor backLeft;
  private final GobildaMotor backRight;

  // Last drive() call, kept for logging. Wheel powers are {FL, FR, BL, BR}.
  private double cmdForward;
  private double cmdRight;
  private double cmdRotate;
  private boolean fieldCentric;
  private double headingRadians;
  private final double[] wheelPowers = new double[4];

  public MecanumDriveSubsystem(HardwareMap hardwareMap) {
    // Set so that a positive power on every motor drives the robot forward.
    // From the Mark 3 log of 27 Oct 17:51: with the left side forward and the
    // right side reversed, a forward stick spun the robot counterclockwise and
    // a clockwise rotate drove it backward (Pinpoint pods), so the left side
    // was inverted. DriveFL's encoder counts against its motor, so it reads
    // backward while the wheel drives forward.
    frontLeft = createDriveMotor(hardwareMap, "DriveFL", true);
    frontRight = createDriveMotor(hardwareMap, "DriveFR", true);
    backLeft = createDriveMotor(hardwareMap, "DriveBL", false);
    backRight = createDriveMotor(hardwareMap, "DriveBR", true);
  }

  private static GobildaMotor createDriveMotor(HardwareMap hardwareMap, String name, boolean reversed) {
    return new GobildaMotor(hardwareMap, name, reversed, DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
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
    cmdForward = forward;
    cmdRight = right;
    cmdRotate = rotate;
    this.fieldCentric = fieldCentric;
    this.headingRadians = headingRadians;

    if (fieldCentric) {
      double theta = Math.atan2(forward, right);
      double r = Math.hypot(right, forward);

      theta = AngleUnit.normalizeRadians(theta - headingRadians);

      forward = r * Math.sin(theta);
      right = r * Math.cos(theta);
    }

    mix(forward, right, rotate, wheelPowers);

    frontLeft.setPower(wheelPowers[0]);
    frontRight.setPower(wheelPowers[1]);
    backLeft.setPower(wheelPowers[2]);
    backRight.setPower(wheelPowers[3]);
  }

  /**
   * Robot-centric mecanum mix (gm0), with the rollers forming an X seen from
   * above.
   *
   * @param forward forward power, [-1, 1]
   * @param right   strafe-right power, [-1, 1]
   * @param rotate  clockwise rotation power, [-1, 1]
   * @param out     receives the wheel powers {FL, FR, BL, BR}
   */
  static void mix(double forward, double right, double rotate, double[] out) {
    double frontLeftPower = forward + right + rotate;
    double frontRightPower = forward - right - rotate;
    double backLeftPower = forward - right + rotate;
    double backRightPower = forward + right - rotate;

    // Scale all wheels down together so no power exceeds 1 and the direction
    // of travel is preserved.
    double maxPower = Math.max(1.0,
        Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

    out[0] = frontLeftPower / maxPower;
    out[1] = frontRightPower / maxPower;
    out[2] = backLeftPower / maxPower;
    out[3] = backRightPower / maxPower;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Drive/Cmd/Forward", cmdForward);
    Logger.recordOutput("Drive/Cmd/Right", cmdRight);
    Logger.recordOutput("Drive/Cmd/Rotate", cmdRotate);
    Logger.recordOutput("Drive/FieldCentric", fieldCentric);
    Logger.recordOutput("Drive/HeadingRad", headingRadians);
    Logger.recordOutput("Drive/WheelPowers", wheelPowers.clone());
  }

  /** Cuts power to all four wheels. */
  public void stop() {
    Arrays.fill(wheelPowers, 0.0);
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
