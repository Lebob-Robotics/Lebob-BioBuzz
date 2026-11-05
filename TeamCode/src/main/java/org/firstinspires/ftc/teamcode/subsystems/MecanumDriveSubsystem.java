package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;
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
  // Picked in a simulation of the robot's turning fitted to TeleOp logs: turn rate settles 0.12 s after a power
  // change, 6.3 rad/s per unit turn power, and the power lands 60-120 ms after the heading it was worked out
  // from. KP 3 with KD 0.18 swung 40-70 degrees either side of a still target at about 1 Hz, as the model
  // predicts with that delay (5 Nov, 093250). KP 1 overshot by 10-20 degrees; judging the error
  // HEADING_LATENCY_S ahead held a still target within 8 degrees throughout the simulation.
  public static double HEADING_KP = 1.0;
  public static double HEADING_KD = 0.06; // On the error's rate, so it doesn't brake a turn that follows the target
  public static double HEADING_LATENCY_S = 0.08; // The error is judged this far ahead, where the power will act
  // Turn power per rad/s the target heading moves. Strafing near the Cell swings the aimed heading by up to
  // ~80 deg/s, and P alone trailed it by 10-20 degrees (5 Nov, 084358).
  public static double HEADING_KV = 0.16;
  // Least turn power while off by more than HEADING_KS_ABOVE_RAD. Standing still, 0.10 didn't turn the robot
  // and 0.15 did, so 0.12 left it stuck 7-9 degrees off, outside the shot tolerance (5 Nov, 084358).
  public static double HEADING_KS = 0.16;
  public static double HEADING_KS_ABOVE_RAD = Math.toRadians(2.5);
  // While the robot pushes at the least power and doesn't turn, that power grows at this rate, up to HEADING_KS
  // plus HEADING_KS_BOOST_MAX, and drops back once it turns. 0.16 left it 8.6 degrees off for 4.5 s, wheels at
  // 0.16 and 12.2 V, where 0.15 had turned it the run before (5 Nov, 083302)
  public static double HEADING_KS_BOOST_PER_S = 0.2;
  public static double HEADING_KS_BOOST_MAX = 0.15;
  public static double HEADING_STUCK_RATE_RAD_S = Math.toRadians(5); // Turning slower than this counts as stuck
  public static double HEADING_MAX_ROTATE = 0.8;
  // While the heading lock turns, the drive gives way first, down to this share of the stick: at full stick
  // the shared scale-down left the lock 0.28 of the 0.48 it asked for, and the aim trailed by 15 degrees.
  public static double LOCK_MIN_TRANSLATION_SHARE = 0.4;
  // Off by more than this, the lock keeps turning the way it already was: right at 180 degrees the shorter
  // way flips with each reading, and the robot wiggled in place for 0.7 s instead of turning (5 Nov, 092433).
  private static final double KEEP_TURN_DIRECTION_ABOVE_RAD = Math.toRadians(150);
  // Stick response. Inside STICK_DEADBAND the output is 0. Past it the output starts at STICK_MIN_POWER,
  // about where the wheels start to turn, and curves up to 1 by STICK_EXPONENT. Without the start power,
  // stick^2 stayed under the ~0.2-0.3 the wheels need until about half stick, then lurched (4 Nov logs:
  // wheels mostly still below 0.25 power).
  public static double STICK_EXPONENT = 2.0;
  public static double STICK_DEADBAND = 0.05;
  public static double STICK_MIN_POWER = 0.18;
  // The turn stick's deadband. With the drive stick pushed hard, the turn stick read 0.06-0.11 untouched,
  // which STICK_DEADBAND turned into STICK_MIN_POWER: with the wheels already rolling that spun the robot at
  // 60-115 deg/s while the driver went straight (5 Nov, 093520).
  public static double TURN_STICK_DEADBAND = 0.15;
  // Least turn power of the heading hold. 0: it only acts while the robot translates, when the wheels are
  // already rolling, and a least power there made it overshoot back and forth.
  public static double HEADING_HOLD_KS = 0.0;
  /**
   * Speed control needs every drive encoder counting the same way as its motor. On 27 Oct DriveFL's
   * counted backward, which under speed control makes the wheel run away at full speed. Set true
   * once Drive Check shows all four encoders count forward when the robot drives forward.
   */
  private static final boolean DRIVE_USES_ENCODERS = false;

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
  private boolean headingHoldActive;
  private double headingTargetRad;
  private double headingErrorRad;
  private boolean lockedLastCall;
  private double ksBoost; // Added to HEADING_KS while stuck
  private double lastLockS;
  private final ElapsedTime clock = new ElapsedTime();

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
    return DRIVE_USES_ENCODERS
        ? GobildaMotor.speedControlled(hardwareMap, name, reversed, false)
        : GobildaMotor.powerControlled(hardwareMap, name, reversed, false);
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
    lockedLastCall = false;
    drive(forward, right, rotate, fieldCentric, headingRadians, false);
  }

  private void drive(double forward, double right, double rotate, boolean fieldCentric, double headingRadians,
      boolean turnFirst) {
    cmdForward = forward;
    cmdRight = right;
    cmdRotate = rotate;
    this.fieldCentric = fieldCentric;
    this.headingRadians = headingRadians;
    headingLockActive = false;
    headingHoldActive = false;

    if (fieldCentric) {
      double theta = Math.atan2(forward, right);
      double r = Math.hypot(right, forward);

      theta = AngleUnit.normalizeRadians(theta - headingRadians);

      forward = r * Math.sin(theta);
      right = r * Math.cos(theta);
    }

    if (turnFirst) {
      mixTurnFirst(forward, right, rotate, LOCK_MIN_TRANSLATION_SHARE, wheelPowers);
    } else {
      mix(forward, right, rotate, wheelPowers);
    }
    for (int i = 0; i < wheelPowers.length; i++) {
      wheelPowers[i] *= GobildaMotor.MAX_OUTPUT; // Full stick reaches the motor cap on every wheel together, so the direction stays exact
    }

    frontLeft.setPower(wheelPowers[0]);
    frontRight.setPower(wheelPowers[1]);
    backLeft.setPower(wheelPowers[2]);
    backRight.setPower(wheelPowers[3]);
  }

  /**
   * Curves one stick axis: 0 inside the deadband, then from STICK_MIN_POWER up to 1, gentle at first.
   * Sign kept; 1 stays 1.
   */
  public static double shapeStick(double stick) {
    return shapeStick(stick, 1.0);
  }

  /** As above, but full stick gives maxPower. Just past the deadband is still STICK_MIN_POWER, so the robot moves. */
  public static double shapeStick(double stick, double maxPower) {
    return shapeStick(stick, maxPower, STICK_DEADBAND);
  }

  /** As {@link #shapeStick(double)}, for the turn stick, with TURN_STICK_DEADBAND. */
  public static double shapeTurnStick(double stick) {
    return shapeStick(stick, 1.0, TURN_STICK_DEADBAND);
  }

  private static double shapeStick(double stick, double maxPower, double deadband) {
    double pushed = Math.min(1.0, Math.abs(stick));
    if (pushed <= deadband) {
      return 0.0;
    }
    double past = (pushed - deadband) / (1.0 - deadband); // 0 at the deadband, 1 at full stick
    double top = Math.max(STICK_MIN_POWER, maxPower);
    return Math.copySign(STICK_MIN_POWER + (top - STICK_MIN_POWER) * Math.pow(past, STICK_EXPONENT), stick);
  }

  /**
   * Curves the drive stick by how far it is pushed, not per axis, so the direction of travel stays the
   * one the stick points. Returns {forward, right}.
   */
  public static double[] shapeTranslation(double forward, double right) {
    return shapeTranslation(forward, right, 1.0);
  }

  /** As above, with full stick giving maxPower. */
  public static double[] shapeTranslation(double forward, double right, double maxPower) {
    double magnitude = Math.hypot(forward, right);
    if (magnitude < 1e-9) {
      return new double[] {0, 0};
    }
    double scale = shapeStick(Math.min(1.0, magnitude), maxPower) / magnitude;
    return new double[] {forward * scale, right * scale};
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

  /**
   * As {@link #mix}, but past full power the translation is scaled down first, to no less than
   * minTranslationShare of itself, so the rotation keeps its power. Anything still over 1 is then scaled
   * down together.
   */
  static void mixTurnFirst(double forward, double right, double rotate, double minTranslationShare, double[] out) {
    double[] translation = {forward + right, forward - right, forward - right, forward + right};
    double[] turn = {rotate, -rotate, rotate, -rotate};
    double share = 1.0;
    for (int i = 0; i < 4; i++) {
      double t = translation[i];
      if (Math.abs(t) > 1e-12) {
        // Largest share s with |s * t + turn| <= 1
        double limit = ((t > 0 ? 1.0 : -1.0) - turn[i]) / t;
        share = Math.min(share, limit);
      }
    }
    share = Math.max(minTranslationShare, Math.min(1.0, share));
    mix(forward * share, right * share, rotate, out);
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
    driveHeadingLock(forward, right, targetHeadingRad, 0.0, currentHeadingRad, headingRateRadPerSec);
  }

  /**
   * As above, for a target that moves.
   *
   * @param targetRateRadPerSec how fast the target heading is moving, CCW positive
   */
  public void driveHeadingLock(double forward, double right, double targetHeadingRad, double targetRateRadPerSec,
      double currentHeadingRad, double headingRateRadPerSec) {
    double error = keepTurnDirection(AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad),
        lockedLastCall ? headingErrorRad : Double.NaN);
    double nowS = clock.seconds();
    ksBoost = lockedLastCall ? stuckBoost(ksBoost, error, headingRateRadPerSec, nowS - lastLockS) : 0.0;
    lastLockS = nowS;
    double ccw = headingLockTurn(error, targetRateRadPerSec, headingRateRadPerSec, HEADING_KS + ksBoost);

    // drive()'s rotate is clockwise positive, the opposite of heading.
    drive(forward, right, -ccw, true, currentHeadingRad, true);

    // Set after drive(), which clears the flag.
    lockedLastCall = true;
    headingLockActive = true;
    headingTargetRad = targetHeadingRad;
    headingErrorRad = error;
  }

  /**
   * Field-centric drive that holds a still heading while translating: the heading lock's PD without its least
   * power or stuck boost (HEADING_HOLD_KS).
   */
  public void driveHeadingHold(double forward, double right, double targetHeadingRad, double currentHeadingRad,
      double headingRateRadPerSec) {
    double error = AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad);
    double ccw = headingLockTurn(error, 0.0, headingRateRadPerSec, HEADING_HOLD_KS);
    lockedLastCall = false;
    drive(forward, right, -ccw, true, currentHeadingRad, true);
    headingHoldActive = true;
    headingTargetRad = targetHeadingRad;
    headingErrorRad = error;
  }

  /**
   * The error to turn by, given the shorter-way error now and the one used last loop (NaN if none): near
   * 180 degrees off it stays on the same side as last loop, so the robot keeps turning one way.
   */
  static double keepTurnDirection(double errorRad, double lastErrorRad) {
    if (Double.isNaN(lastErrorRad) || Math.abs(lastErrorRad) < KEEP_TURN_DIRECTION_ABOVE_RAD
        || Math.signum(errorRad) == Math.signum(lastErrorRad)) {
      return errorRad;
    }
    double otherWay = errorRad - Math.copySign(2 * Math.PI, errorRad);
    return Math.abs(otherWay) < KEEP_TURN_DIRECTION_ABOVE_RAD + Math.PI / 2 ? otherWay : errorRad;
  }

  /**
   * The next loop's extra least turn power: grows while off by more than HEADING_KS_ABOVE_RAD and not turning,
   * otherwise 0. A long loop counts as at most 0.1 s, so a pause doesn't add a big step.
   */
  static double stuckBoost(double boost, double errorRad, double headingRateRadPerSec, double dtS) {
    if (Math.abs(errorRad) <= HEADING_KS_ABOVE_RAD || Math.abs(headingRateRadPerSec) > HEADING_STUCK_RATE_RAD_S) {
      return 0.0;
    }
    double dt = Math.max(0.0, Math.min(0.1, dtS));
    return Math.min(HEADING_KS_BOOST_MAX, boost + HEADING_KS_BOOST_PER_S * dt);
  }

  /** Heading lock turn power, CCW positive, clamped to HEADING_MAX_ROTATE. */
  static double headingLockTurn(double errorRad, double targetRateRadPerSec, double headingRateRadPerSec) {
    return headingLockTurn(errorRad, targetRateRadPerSec, headingRateRadPerSec, HEADING_KS);
  }

  /** As above, with the given least turn power. */
  static double headingLockTurn(double errorRad, double targetRateRadPerSec, double headingRateRadPerSec,
      double ks) {
    double closingRate = targetRateRadPerSec - headingRateRadPerSec;
    double predictedErrorRad = errorRad + closingRate * HEADING_LATENCY_S;
    double ccw = HEADING_KP * predictedErrorRad + HEADING_KV * targetRateRadPerSec + HEADING_KD * closingRate;
    if (Math.abs(predictedErrorRad) > HEADING_KS_ABOVE_RAD && Math.abs(ccw) < ks
        && Math.signum(ccw) == Math.signum(predictedErrorRad)) {
      ccw = Math.copySign(ks, predictedErrorRad);
    }
    return Math.max(-HEADING_MAX_ROTATE, Math.min(HEADING_MAX_ROTATE, ccw));
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
    Logger.recordOutput("Drive/HeadingHold/Active", headingHoldActive);
    Logger.recordOutput("Drive/HeadingLock/TargetRad", headingTargetRad);
    Logger.recordOutput("Drive/HeadingLock/ErrorRad", headingErrorRad);
    Logger.recordOutput("Drive/HeadingLock/KsBoost", ksBoost);
    Logger.recordOutput("Drive/WheelPowers", wheelPowers.clone());
  }

  /** Cuts power to all four wheels. */
  public void stop() {
    lockedLastCall = false;
    Arrays.fill(wheelPowers, 0.0);
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}
