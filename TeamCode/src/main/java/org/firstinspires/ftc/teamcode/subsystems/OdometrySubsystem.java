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
 * reports the robot's field pose.
 */
public class OdometrySubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  // The SDK javadoc only says "X (forward) velocity", so the frame is unconfirmed.
  private static final boolean PINPOINT_VELOCITY_IS_FIELD_FRAME = true; // TODO verify

  private final GoBildaPinpointDriver pinpoint;

  public OdometrySubsystem(HardwareMap hardwareMap) {
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
  }

  /**
   * Called once from the OpMode's init(). Configures the Pinpoint and zeroes its
   * pose.
   */
  public void init() {
    // From the 25 Oct CAD, relative to the robot centre (the middle of the four mecanum wheels),
    // which the shooter and camera positions in ShootOnTheMoveConstants also use: the X pod is
    // 71.5 mm right of centre and the Y pod is on the centre line.
    // TODO replace with the Odometry Calibration OpMode result.
    pinpoint.setOffsets(-71.5, 0.1, DistanceUnit.MM);
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
    return pinpoint.getPosition();
  }

  /**
   * Velocity in the field frame.
   *
   * @return {vx m/s, vy m/s, omega rad/s}
   */
  public double[] getVelocityMetersPerSec() {
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

  /** Overwrites the tracked pose, e.g. to start from a known field position. */
  public void setPose(double xMeters, double yMeters, double headingRad) {
    pinpoint.setPosition(new Pose2D(
        DistanceUnit.METER, xMeters, yMeters, AngleUnit.RADIANS, headingRad));
  }

  /** Sets the reported heading in place, keeping the current position. */
  public void setHeading(double headingRad) {
    Pose2D current = pinpoint.getPosition();
    pinpoint.setPosition(new Pose2D(
        DistanceUnit.MM, current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM),
        AngleUnit.RADIANS, headingRad));
  }

  /** Zeroes the reported heading in place, keeping the current position. */
  public void resetHeading() {
    Pose2D current = pinpoint.getPosition();
    pinpoint.setPosition(new Pose2D(
        DistanceUnit.MM, current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM),
        AngleUnit.DEGREES, 0));
  }
}
