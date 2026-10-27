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
 * On Mark 3 the Pinpoint works in a mirror image of that frame. Measured with
 * the Drive Check OpMode on 27 Oct: pushing the robot forward raised its Y,
 * pushing it left raised its X, and a quarter turn counterclockwise read -89
 * degrees. So the pod on the X port measures strafe, the pod on the Y port
 * measures forward, and the board's IMU reads clockwise as positive (probably
 * mounted upside down). The mirror image is self-consistent, so the Pinpoint
 * still tracks correctly in it, and {@link #toRobotFrame} reflects every pose
 * we read back into the robot's frame. Swapping the two pod cables and
 * flipping the board over would remove the need for it.
 */
public class OdometrySubsystem extends SubsystemBase {
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
    // In the Pinpoint's mirrored frame (see the class comment), the X offset is
    // how far forward the strafe pod sits and the Y offset is how far left the
    // forward pod sits: forward pod on the centre line, strafe pod 105 mm
    // behind the centre.
    pinpoint.setOffsets(-105.0, 0.0, DistanceUnit.MM);
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

    Pose2D pose = getPose();
    double headingRad = pose.getHeading(AngleUnit.RADIANS);

    // Meters and radians, as AdvantageScope expects.
    Logger.recordOutput("Odometry/Pose", new Pose2d(
        pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), new Rotation2d(headingRad)));

    // Same pose in the units we read it in on the field.
    Logger.recordOutput("Odometry/XInches", pose.getX(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/YInches", pose.getY(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/HeadingDegrees", pose.getHeading(AngleUnit.DEGREES));

    // Reflected like the pose: X and Y swap, and the heading rate flips sign.
    Logger.recordOutput("Odometry/Velocity/XInchesPerSec", pinpoint.getVelY(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/Velocity/YInchesPerSec", pinpoint.getVelX(DistanceUnit.INCH));
    Logger.recordOutput("Odometry/Velocity/HeadingDegreesPerSec",
        -pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
    Logger.recordOutput("Odometry/DeviceStatus", String.valueOf(pinpoint.getDeviceStatus()));
  }

  public Pose2D getPose() {
    return pinpoint == null ? ZERO_POSE : toRobotFrame(pinpoint.getPosition());
  }

  /**
   * Reflects a pose between the Pinpoint's mirrored frame and the robot's
   * frame: X and Y swap and the heading changes sign. Its own inverse, so it
   * also converts a robot-frame pose for {@code setPosition}.
   */
  public static Pose2D toRobotFrame(Pose2D pose) {
    return new Pose2D(DistanceUnit.MM,
        pose.getY(DistanceUnit.MM), pose.getX(DistanceUnit.MM),
        AngleUnit.RADIANS, -pose.getHeading(AngleUnit.RADIANS));
  }

  /**
   * Zeroes the reported heading in place, keeping the current position. Works
   * on the Pinpoint's own pose: a zero heading is zero in both frames.
   */
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
