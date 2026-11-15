package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * Wraps a goBILDA Pinpoint Odometry Computer reading two dead-wheel pods, and
 * reports the robot's field pose: X forward, Y left, heading counterclockwise.
 *
 * <p>
 * The forward pod must be on the Pinpoint's X port and the strafe pod on its Y
 * port. On 27 Oct Mark 3 had them the other way round: pushing the robot
 * forward raised Y and pushing it left raised X. The heading comes from the
 * Pinpoint's IMU, so field-centric driving still works with the pods swapped,
 * but X and Y are wrong once the robot turns.
 */
public class OdometrySubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  // The SDK javadoc only says "X (forward) velocity".
  private static final boolean PINPOINT_VELOCITY_IS_FIELD_FRAME = true; // Checked 4 Nov: facing 180, field-frame velocity matched the pose change to 4 in/s, robot frame was 30 out

  private static final Pose2D ZERO_POSE =
      new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

  /** Null when "pinpoint" is not in the config: the pose then stays at zero. */
  private final GoBildaPinpointDriver pinpoint;
  /** The last pose read while the Pinpoint said READY, in its own frame. */
  private Pose2D lastGoodPose = ZERO_POSE;
  /**
   * Field pose = the Pinpoint's pose turned by offsetHeadingRad, then shifted by (offsetXM, offsetYM). Corrections
   * change these, never the Pinpoint: writing a pose into it threw away whatever it turned between the loop's read
   * and the write, 1.1 degrees a write turning, and the heading was 23 degrees out after 21 corrections (5 Nov, 081521).
   */
  private double offsetXM;
  private double offsetYM;
  private double offsetHeadingRad;
  /**
   * Bad reads for this long and the pose is no longer trusted. Single bad reads clear within a loop or two (60 ms,
   * 5 Nov); after a reset the Pinpoint stayed on FAULT_BAD_READ, the pose froze while the robot drove on, and the
   * field test, seeing no progress, pushed it off the field (5 Nov, 085633).
   */
  private static final double FAULT_AFTER_S = 0.3;
  /** While faulted, the configuration and the last good pose are written again this often. */
  private static final double RECONFIGURE_EVERY_S = 1.0;
  private long badSinceNanos = -1;
  /** A read was bad since the last good one: the Pinpoint may have reset in between. */
  private boolean sawBadRead;
  private int resetsCaught;
  /**
   * A good read this near zero, after bad reads and a jump from the read before them, is the Pinpoint having reset.
   * Loose, as the robot may already be moving: 223434's first read after the reset was 3 degrees turned.
   */
  static final double RESET_NEAR_ZERO_M = 0.10;
  static final double RESET_NEAR_ZERO_RAD = Math.toRadians(10);
  static final double RESET_JUMP_M = 0.15;
  static final double RESET_JUMP_RAD = Math.toRadians(15);
  private long lastReconfigureNanos;
  private int reconfigures;

  public OdometrySubsystem(HardwareMap hardwareMap) {
    GoBildaPinpointDriver found = null;
    try {
      found = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    } catch (IllegalArgumentException e) {
      // Not configured; run without odometry.
    }
    pinpoint = found;
  }

  /** True when the Pinpoint is configured. */
  public boolean isAvailable() {
    return pinpoint != null;
  }

  /**
   * Called once from the OpMode's init(). Configures the Pinpoint and zeroes its
   * pose.
   */
  public void init() {
    if (pinpoint == null) {
      return;
    }
    configure();
    // Robot must be stationary during this call.
    pinpoint.resetPosAndIMU();
    lastGoodPose = ZERO_POSE;
    offsetXM = 0;
    offsetYM = 0;
    offsetHeadingRad = 0;
  }

  /** Offsets, pods and directions: the Pinpoint forgets them if it resets. */
  private void configure() {
    // CAD, to the middle of the four wheels, the centre ShootOnTheMoveConstants uses. The old (0, -105) made
    // spinning in place read as 0.07 m/s forward and 0.10 m/s left per rad/s (fits 0.85-0.99 over four runs),
    // which worked back to offsets of about (-65, 0): the hooded shot's lead swung with every turn (5 Nov, 084206)
    pinpoint.setOffsets(-71.5, 0.1, DistanceUnit.MM);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    // X (forward) pod counts backward: TeleOp log 3 Nov 22:22, wheels driving the robot backward moved
    // the pose forward (30 of 31 samples). Y (strafe) counted backward too: pushing the robot right raised Y (4 Nov).
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.REVERSED,
        GoBildaPinpointDriver.EncoderDirection.REVERSED);
  }

  /**
   * Logs the pose. When PsiKit is running, its Pinpoint logger calls
   * pinpoint.update() once at the start of every loop (see
   * Main#onPsiKitConfigureLogging), so getPosition() is already fresh and is
   * exactly what gets logged and replayed. Without PsiKit we update here.
   */
  @Override
  public void periodic() {
    if (pinpoint == null) {
      return;
    }
    if (!Logger.isRunning()) {
      pinpoint.update();
    }

    // A FAULT_BAD_READ comes back as garbage for a loop; log the pose the robot uses, not a 0.4 m spike
    Pose2D pose = getPose();
    double headingRad = pose.getHeading(AngleUnit.RADIANS);

    // Meters and radians, as AdvantageScope expects.
    Logger.recordOutput("Odometry/Pose", new Pose2d(
        pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), new Rotation2d(headingRad)));
    Logger.recordOutput("Odometry/Offset", new double[] {offsetXM, offsetYM, offsetHeadingRad});

    // Same pose in the units we read it in on the field.
    Logger.recordOutput("Odometry/XInches", pose.getX(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/YInches", pose.getY(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/HeadingDegrees", pose.getHeading(AngleUnit.DEGREES));

    double[] fieldVelocity = getVelocityMetersPerSec();
    Logger.recordOutput("Odometry/Velocity/XInchesPerSec", fieldVelocity[0] / 0.0254);
    Logger.recordOutput("Odometry/Velocity/YInchesPerSec", fieldVelocity[1] / 0.0254);
    Logger.recordOutput("Odometry/Velocity/HeadingDegreesPerSec",
        pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    Logger.recordOutput("Odometry/VelocityField", new double[] { fieldVelocity[0], fieldVelocity[1] });
    Logger.recordOutput("Odometry/DeviceStatus", String.valueOf(pinpoint.getDeviceStatus()));

    long now = System.nanoTime();
    if (isReadGood()) {
      badSinceNanos = -1;
    } else if (badSinceNanos < 0) {
      badSinceNanos = now;
    } else if (isFaulted() && (now - lastReconfigureNanos) / 1e9 >= RECONFIGURE_EVERY_S) {
      lastReconfigureNanos = now;
      reconfigures++;
      configure();
      pinpoint.setPosition(lastGoodPose);
    }
    Logger.recordOutput("Odometry/Faulted", isFaulted());
    Logger.recordOutput("Odometry/Reconfigures", reconfigures);
    Logger.recordOutput("Odometry/ResetsCaught", resetsCaught);
  }

  /** True once the Pinpoint has read badly for {@link #FAULT_AFTER_S}: its pose is stale, however it looks. */
  public boolean isFaulted() {
    return badSinceNanos >= 0 && (System.nanoTime() - badSinceNanos) / 1e9 >= FAULT_AFTER_S;
  }

  /**
   * The pose. When the Pinpoint's last read was bad this is the last good pose instead: a NOT_READY read
   * comes back as all zeros, and writing a correction based on it into the Pinpoint loses the heading
   * (4 Nov, 357 s: heading -172 went to 0).
   */
  public Pose2D getPose() {
    if (pinpoint == null) {
      return ZERO_POSE;
    }
    readDevice();
    return toField(lastGoodPose);
  }

  /**
   * Takes a good read as the last good pose. If the Pinpoint came back from bad reads at zero, it reset (the
   * reset init() asks for lands late, or power dipped): the offset is redone so the field pose carries on where it
   * was. The robot holds still while the Pinpoint calibrates. 13 Nov 223434: the reset from INIT finished 0.17 s
   * after START, the start pose had been set against the old reading, and the pose jumped 0.45 m and 32 degrees,
   * so the Scoring Auto drove off the field.
   */
  private void readDevice() {
    if (!isReadGood()) {
      sawBadRead = true;
      return;
    }
    Pose2D now = pinpoint.getPosition();
    if (sawBadRead && cameBackReset(raw(lastGoodPose), raw(now))) {
      Pose2D field = toField(lastGoodPose);
      lastGoodPose = now;
      setPose(field.getX(DistanceUnit.METER), field.getY(DistanceUnit.METER), field.getHeading(AngleUnit.RADIANS));
      resetsCaught++;
    }
    sawBadRead = false;
    lastGoodPose = now;
  }

  private static double[] raw(Pose2D p) {
    return new double[] {p.getX(DistanceUnit.METER), p.getY(DistanceUnit.METER), p.getHeading(AngleUnit.RADIANS)};
  }

  /** {x, y, heading} in the Pinpoint's frame: before the bad reads, and now. */
  static boolean cameBackReset(double[] before, double[] now) {
    boolean atZero = Math.hypot(now[0], now[1]) < RESET_NEAR_ZERO_M && Math.abs(normalize(now[2])) < RESET_NEAR_ZERO_RAD;
    boolean jumped = Math.hypot(now[0] - before[0], now[1] - before[1]) > RESET_JUMP_M
        || Math.abs(normalize(now[2] - before[2])) > RESET_JUMP_RAD;
    return atZero && jumped;
  }

  private Pose2D toField(Pose2D device) {
    double x = device.getX(DistanceUnit.METER);
    double y = device.getY(DistanceUnit.METER);
    double cos = Math.cos(offsetHeadingRad);
    double sin = Math.sin(offsetHeadingRad);
    return new Pose2D(DistanceUnit.METER, offsetXM + x * cos - y * sin, offsetYM + x * sin + y * cos,
        AngleUnit.RADIANS, normalize(device.getHeading(AngleUnit.RADIANS) + offsetHeadingRad));
  }

  private static double normalize(double rad) {
    return Math.atan2(Math.sin(rad), Math.cos(rad));
  }

  /** False when the Pinpoint's last update did not read it as READY, so its pose must not be used or written back. */
  public boolean isReadGood() {
    return pinpoint == null || pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
  }

  /**
   * Velocity in the field frame.
   *
   * @return {vx m/s, vy m/s, omega rad/s}
   */
  public double[] getVelocityMetersPerSec() {
    if (pinpoint == null) {
      return new double[] { 0, 0, 0 };
    }
    double vx = pinpoint.getVelX(DistanceUnit.METER);
    double vy = pinpoint.getVelY(DistanceUnit.METER);
    double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

    // To the Pinpoint's own frame first if it reports robot-frame velocity, then turned into the field frame
    double turn = offsetHeadingRad
        + (PINPOINT_VELOCITY_IS_FIELD_FRAME ? 0 : pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
    double cos = Math.cos(turn);
    double sin = Math.sin(turn);
    return new double[] { vx * cos - vy * sin, vx * sin + vy * cos, omega };
  }

  /**
   * Makes the pose read this loop report as the given one, from now on. Motion since that read is kept, because
   * only the offset changes.
   */
  public void setPose(double xM, double yM, double headingRad) {
    if (pinpoint == null) {
      return;
    }
    double x = lastGoodPose.getX(DistanceUnit.METER); // Not a bad read's zeros
    double y = lastGoodPose.getY(DistanceUnit.METER);
    offsetHeadingRad = normalize(headingRad - lastGoodPose.getHeading(AngleUnit.RADIANS));
    double cos = Math.cos(offsetHeadingRad);
    double sin = Math.sin(offsetHeadingRad);
    offsetXM = xM - (x * cos - y * sin);
    offsetYM = yM - (x * sin + y * cos);
  }

  /** Sets the reported heading in place, keeping the current position. */
  public void setHeading(double headingRad) {
    if (pinpoint == null) {
      return;
    }
    Pose2D current = getPose();
    setPose(current.getX(DistanceUnit.METER), current.getY(DistanceUnit.METER), headingRad);
  }
}
