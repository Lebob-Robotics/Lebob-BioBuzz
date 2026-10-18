package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class MecanumDriveSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  // Heading lock gains. The rotate sign is to be verified on the robot.
  public static double HEADING_KP = 1.0; // TODO tune on robot
  public static double HEADING_KD = 0.05; // TODO tune on robot
  public static double HEADING_MAX_ROTATE = 0.8; // TODO tune on robot

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

  // Heading lock state, kept for logging. drive() clears the active flag.
  private boolean headingLockActive;
  private double headingTargetRad;
  private double headingErrorRad;

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
    cmdForward = forward;
    cmdRight = right;
    cmdRotate = rotate;
    this.fieldCentric = fieldCentric;
    this.headingRadians = headingRadians;
    headingLockActive = false;

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

    wheelPowers[0] = frontLeftPower / maxPower;
    wheelPowers[1] = frontRightPower / maxPower;
    wheelPowers[2] = backLeftPower / maxPower;
    wheelPowers[3] = backRightPower / maxPower;

    frontLeft.setPower(wheelPowers[0]);
    frontRight.setPower(wheelPowers[1]);
    backLeft.setPower(wheelPowers[2]);
    backRight.setPower(wheelPowers[3]);
  }

  /**
   * Field-centric drive that turns to hold a target heading instead of taking
   * a rotate input. A PD loop on heading error; the sign is to be verified on
   * the robot.
   *
   * @param forward              forward power, [-1, 1], relative to the field
   * @param right                strafe-right power, [-1, 1], relative to the field
   * @param targetHeadingRad     desired field heading, CCW positive
   * @param currentHeadingRad    current field heading, CCW positive (Pinpoint)
   * @param headingRateRadPerSec current turn rate, CCW positive
   */
  public void driveHeadingLock(double forward, double right, double targetHeadingRad,
      double currentHeadingRad, double headingRateRadPerSec) {
    double error = AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad);
    double ccw = HEADING_KP * error - HEADING_KD * headingRateRadPerSec;
    ccw = Math.max(-HEADING_MAX_ROTATE, Math.min(HEADING_MAX_ROTATE, ccw));

    // drive()'s rotate is clockwise positive, the opposite of heading.
    drive(forward, right, -ccw, true, currentHeadingRad);

    // Set after drive(), which clears the flag.
    headingLockActive = true;
    headingTargetRad = targetHeadingRad;
    headingErrorRad = error;
  }

  /** Heading error from the last {@link #driveHeadingLock}, CCW positive. */
  public double getHeadingErrorRad() {
    return headingErrorRad;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Drive/Cmd/Forward", cmdForward);
    Logger.recordOutput("Drive/Cmd/Right", cmdRight);
    Logger.recordOutput("Drive/Cmd/Rotate", cmdRotate);
    Logger.recordOutput("Drive/FieldCentric", fieldCentric);
    Logger.recordOutput("Drive/HeadingRad", headingRadians);
    Logger.recordOutput("Drive/HeadingLock/Active", headingLockActive);
    Logger.recordOutput("Drive/HeadingLock/TargetRad", headingTargetRad);
    Logger.recordOutput("Drive/HeadingLock/ErrorRad", headingErrorRad);
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
