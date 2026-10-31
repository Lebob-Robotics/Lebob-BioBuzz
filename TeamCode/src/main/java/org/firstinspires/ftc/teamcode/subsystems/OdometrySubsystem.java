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
  // The SDK javadoc only says "X (forward) velocity", so the frame is unconfirmed.
  private static final boolean PINPOINT_VELOCITY_IS_FIELD_FRAME = true; // TODO verify

  private static final Pose2D ZERO_POSE =
      new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

  /** Null when "pinpoint" is not in the config: the pose then stays at zero. */
  private final GoBildaPinpointDriver pinpoint;

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
    // Andre's values, which the robot has been driving on. CAD gives (-71.5, 0.1) to the middle of the
    // four wheels, the centre ShootOnTheMoveConstants uses. TODO replace with the Odometry Calibration
    // result once the pods are on the right ports.
    pinpoint.setOffsets(0.0, -105.0, DistanceUnit.MM);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Robot must be stationary during this call.
    pinpoint.resetPosAndIMU();
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

    Pose2D pose = pinpoint.getPosition();
    double headingRad = pose.getHeading(AngleUnit.RADIANS);

    // Meters and radians, as AdvantageScope expects.
    Logger.recordOutput("Odometry/Pose", new Pose2d(
        pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), new Rotation2d(headingRad)));

    // Same pose in the units we read it in on the field.
    Logger.recordOutput("Odometry/XInches", pose.getX(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/YInches", pose.getY(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/HeadingDegrees", pose.getHeading(AngleUnit.DEGREES));

    Logger.recordOutput("Odometry/Velocity/XInchesPerSec", pinpoint.getVelX(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/Velocity/YInchesPerSec", pinpoint.getVelY(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/Velocity/HeadingDegreesPerSec",
        pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    double[] velocity = getVelocityMetersPerSec();
    Logger.recordOutput("Odometry/VelocityField", new double[] { velocity[0], velocity[1] });
    Logger.recordOutput("Odometry/DeviceStatus", String.valueOf(pinpoint.getDeviceStatus()));
  }

  public Pose2D getPose() {
    return pinpoint == null ? ZERO_POSE : pinpoint.getPosition();
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

    if (!PINPOINT_VELOCITY_IS_FIELD_FRAME) {
      double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      double fieldVx = vx * cos - vy * sin;
      double fieldVy = vx * sin + vy * cos;
      vx = fieldVx;
      vy = fieldVy;
    }
    return new double[] { vx, vy, omega };
  }

  /** Zeroes the reported heading in place, keeping the current position. */
  public void resetHeading() {
    if (pinpoint == null) {
      return;
    }
    Pose2D current = pinpoint.getPosition();
    pinpoint.setPosition(new Pose2D(
        DistanceUnit.MM, current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM),
        AngleUnit.DEGREES, 0));
  }
}
