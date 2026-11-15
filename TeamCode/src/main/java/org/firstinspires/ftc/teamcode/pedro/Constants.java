package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * Pedro Pathing set-up. Pedro works in inches and seconds, field frame as ours: x toward the audience, heading
 * counterclockwise from +x, origin at the middle.
 */
public final class Constants {
  /**
   * True once {@link #foresightConfig} holds measured values. The speeds, decelerations and braking come from the
   * Pinpoint in 13 Nov's logs (what the Foresight Tuner measures, taken from the auto's and TeleOp's own driving);
   * the translational kPs are still guesses, to check against Shoot Auto's park.
   */
  public static final boolean TUNED = true;

  private Constants() {}

  /**
   * Inches. Measured from 13 Nov's logs (sysid over 42 runs), drive motors on BRAKE as Shoot Auto runs them:
   * top speed the mean of 10 readings at full power (p95 forward, the most strafing reached); deceleration and braking
   * from the auto's stops with no power (forward 102 to 142 in/s^2 from over 0.9 m/s; stop = 0.1425 v + 0.00041 v^2,
   * 0.5 in rms over 8 stops), rounded toward braking early. Strafe braking had 2 stops, so it takes forward's: with
   * the power off the two slid about the same. The guesses were float mode's (31 in from 60 in/s against 10 in).
   * Linear braking also carries 0.07 s of control delay (the pose a loop old, the motors' response): it travels at
   * full speed before braking starts. Without it the measured values ran 6.6 cm past in ShootAutoPedroTest's model;
   * with it none, in 3.0 s; 0.12 s and more stopped 4 cm short.
   */
  public static ForesightConfig foresightConfig() {
    return new ForesightConfig(c -> {
      Controller primaryTranslationalForward = Controller.proportional(0.10);
      Controller secondaryTranslationalForward = Controller.proportional(0.05);
      Controller primaryTranslationalLateral = Controller.proportional(0.12);
      Controller secondaryTranslationalLateral = Controller.proportional(0.06);

      c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
      c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

      c.coast.set(Controller.proportionalFeedforward(0.018));
      c.brake.set(Controller.proportionalFeedforward(0.015));

      c.headingFeedback.set(Controller.proportional(1.0));
      c.headingBrakeCoefficients.set(Vector2D.cartesian(0.10, 0.03));

      c.linearBrakeCoefficients.set(Matrix.diag(0.22, 0.23)); // 0.1425 measured + 0.07 s delay, rounded up
      c.quadraticBrakeCoefficients.set(Matrix.diag(0.00045, 0.0005));

      c.maxAchievableForwardVelocity.set(65.0);
      c.maxAchievableStrafeVelocity.set(48.0);
      c.naturalForwardDeceleration.set(120.0);
      c.naturalStrafeDeceleration.set(110.0);
    });
  }

  public static Algorithm algorithm() {
    return new Foresight(foresightConfig());
  }

  /** The Pinpoint as OdometrySubsystem configures it, for the tuners, which run without the Robot. */
  public static PinpointConfig pinpointConfig() {
    return new PinpointConfig(c -> {
      c.name.set("pinpoint");
      c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
      c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
      c.xPodOffset.set(-71.5);
      c.yPodOffset.set(0.1);
      c.offsetUnits.set(DistanceUnit.MM);
      c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
      c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });
  }

  public static Localizer tuningLocalizer(HardwareMap hardwareMap) {
    return new PinpointLocalizer(hardwareMap, pinpointConfig());
  }

  public static Drivetrain tuningDrivetrain(HardwareMap hardwareMap) {
    return new DirectDrivetrain(new MecanumDriveSubsystem(hardwareMap));
  }
}
