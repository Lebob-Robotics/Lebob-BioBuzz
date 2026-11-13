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
   * False while {@link #foresightConfig} holds guesses. Run "Foresight Tuner" (Sloth tuning menu) on the robot,
   * paste the code it prints into foresightConfig, and set this true. The autonomous warns in INIT until then.
   */
  public static final boolean TUNED = false;

  private Constants() {}

  /** Guesses for a 312 RPM mecanum until the tuner has run. Inches. */
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

      c.linearBrakeCoefficients.set(Matrix.diag(0.10, 0.12));
      c.quadraticBrakeCoefficients.set(Matrix.diag(0.007, 0.008));

      c.maxAchievableForwardVelocity.set(55.0);
      c.maxAchievableStrafeVelocity.set(45.0);
      c.naturalForwardDeceleration.set(70.0);
      c.naturalStrafeDeceleration.set(60.0);
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
