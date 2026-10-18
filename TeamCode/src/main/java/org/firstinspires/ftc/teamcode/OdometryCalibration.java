package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * Measures the goBILDA Pinpoint pod offsets by rotating the robot in place.
 *
 * <p>Spinning about the robot centre makes each pod trace a circle whose
 * radius is its offset, so its encoder travel divided by the angle turned is
 * the offset. Using the raw encoders (not the fused pose) means the currently
 * configured offsets do not affect the result.
 *
 * <p>Usage: keep the robot still during INIT. After START press A for an
 * automatic spin, or B to start a manual capture (turn the robot by hand, about
 * its centre, then press X to finish). Y aborts. Paste the result into
 * OdometrySubsystem.init()'s setOffsets.
 *
 * <p>Sign derivation. Pinpoint frame: X forward, Y left, heading CCW positive.
 * The "X" pod measures forward travel and sits at lateral position xOff (left
 * positive). The "Y" pod measures leftward travel and sits at forward position
 * yOff. A point at (px, py) on a body rotating CCW by dTheta moves by
 * (-py * dTheta, px * dTheta). So:
 * <pre>
 *   dEncX = -xOff * dTheta   =&gt; xOff = -dEncX / dTheta
 *   dEncY = +yOff * dTheta   =&gt; yOff =  dEncY / dTheta
 * </pre>
 * Check: a pod right of centre (xOff &lt; 0) moves forward on a CCW turn, so
 * dEncX &gt; 0 and xOff comes out negative. A pod behind centre (yOff &lt; 0)
 * moves right on a CCW turn, so dEncY &lt; 0 and yOff comes out negative. The
 * formulas hold for a CW spin too, since dEnc and dTheta both flip. This
 * assumes the encoder directions are set so X increases forward and Y
 * increases left (as in OdometrySubsystem).
 */
@TeleOp(name = "Odometry Calibration", group = "Tuning")
public class OdometryCalibration extends LinearOpMode {
  // ---- Tuning ----------------------------------------------------------
  private static final int TURNS = 5;
  private static final double SPIN_POWER = 0.3;
  private static final double MIN_TURNS_WARN = 2.0;

  // goBILDA 4-bar pod resolution, from the SDK (GoBildaPinpointDriver).
  private static final double TICKS_PER_MM = 19.89436789;

  // CAD reference, frame origin front-left, X right, Y back (mm).
  private static final double CAD_CENTER_X = 173.2;
  private static final double CAD_CENTER_Y = 228.6;
  private static final double CAD_ALONG_POD_X = 255.0;
  private static final double CAD_ALONG_POD_Y = 285.0;
  private static final double CAD_ACROSS_POD_X = 168.0;
  private static final double CAD_ACROSS_POD_Y = 256.0;

  private enum Mode { IDLE, AUTO, MANUAL, DONE }

  private GoBildaPinpointDriver pinpoint;
  private MecanumDriveSubsystem drive;

  private Mode mode = Mode.IDLE;
  private boolean prevA;
  private boolean prevB;
  private boolean prevX;

  // Unwrapped heading accumulator and capture start values.
  private double lastHeading;
  private double unwrapped;
  private double startHeading;
  private int startEncX;
  private int startEncY;

  // Latest result.
  private boolean haveResult;
  private double turnsDone;
  private double xOffsetMm;
  private double yOffsetMm;

  @Override
  public void runOpMode() {
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    drive = new MecanumDriveSubsystem(hardwareMap);

    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    // Robot must be stationary during this call.
    pinpoint.resetPosAndIMU();
    sleep(500);

    pinpoint.update();
    lastHeading = pinpoint.getHeading(AngleUnit.RADIANS);
    // Read once; this is a separate I2C read, so not done every loop.
    double cfgX = pinpoint.getXOffset(DistanceUnit.MM);
    double cfgY = pinpoint.getYOffset(DistanceUnit.MM);

    telemetry.addLine("Keep the robot still. Press START, then A (auto) or B (manual).");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      pinpoint.update();
      double heading = pinpoint.getHeading(AngleUnit.RADIANS);
      unwrapped += AngleUnit.normalizeRadians(heading - lastHeading);
      lastHeading = heading;

      boolean a = gamepad1.a;
      boolean b = gamepad1.b;
      boolean x = gamepad1.x;
      boolean idle = mode == Mode.IDLE || mode == Mode.DONE;

      if (idle && a && !prevA) {
        beginCapture(Mode.AUTO);
      } else if (idle && b && !prevB) {
        beginCapture(Mode.MANUAL);
      } else if (gamepad1.y) {
        drive.stop();
        mode = Mode.IDLE;
      }
      prevA = a;
      prevB = b;

      if (mode == Mode.AUTO) {
        drive.drive(0, 0, SPIN_POWER, false, 0);
        if (Math.abs(unwrapped - startHeading) >= TURNS * 2 * Math.PI) {
          drive.stop();
          finishCapture();
        }
      } else if (mode == Mode.MANUAL && x && !prevX) {
        finishCapture();
      }
      prevX = x;

      showTelemetry(cfgX, cfgY);
    }
    drive.stop();
  }

  private void beginCapture(Mode newMode) {
    startHeading = unwrapped;
    startEncX = pinpoint.getEncoderX();
    startEncY = pinpoint.getEncoderY();
    haveResult = false;
    mode = newMode;
  }

  private void finishCapture() {
    double dTheta = unwrapped - startHeading;
    turnsDone = dTheta / (2 * Math.PI);
    if (Math.abs(dTheta) > 1e-3) {
      double dEncXmm = (pinpoint.getEncoderX() - startEncX) / TICKS_PER_MM;
      double dEncYmm = (pinpoint.getEncoderY() - startEncY) / TICKS_PER_MM;
      // See the class Javadoc for the signs.
      xOffsetMm = -dEncXmm / dTheta;
      yOffsetMm = dEncYmm / dTheta;
      haveResult = true;
    }
    mode = Mode.DONE;
  }

  private void showTelemetry(double cfgX, double cfgY) {
    // CAD: X pod lateral offset is left positive, so a pod right of centre is
    // negative. Y pod forward offset is forward positive; CAD Y points back.
    double cadX = -(CAD_ALONG_POD_X - CAD_CENTER_X);
    double cadY = -(CAD_ACROSS_POD_Y - CAD_CENTER_Y);

    telemetry.addData("Mode", mode);
    telemetry.addData("Turns", "%.2f", (unwrapped - startHeading) / (2 * Math.PI));
    if (haveResult) {
      telemetry.addData("Measured X offset (mm)", "%.1f", xOffsetMm);
      telemetry.addData("Measured Y offset (mm)", "%.1f", yOffsetMm);
      telemetry.addData("Turns completed", "%.2f", turnsDone);
      if (Math.abs(turnsDone) < MIN_TURNS_WARN) {
        telemetry.addLine("WARNING: under " + MIN_TURNS_WARN
            + " turns, result is unreliable. Rotate more.");
      }
      telemetry.addLine(String.format(
          "PASTE into OdometrySubsystem.init(): pinpoint.setOffsets(%.1f, %.1f, DistanceUnit.MM);",
          xOffsetMm, yOffsetMm));
    }
    telemetry.addData("Configured on Pinpoint (mm)", "X %.1f, Y %.1f", cfgX, cfgY);
    telemetry.addData("CAD expected (mm)", "X %.1f, Y %.1f (assumes CAD robot centre)", cadX, cadY);
    telemetry.addLine("A: auto spin  B: manual capture  X: end manual  Y: abort");
    telemetry.update();
  }
}
