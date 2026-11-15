package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * How the Pinpoint is set up: OdometrySubsystem configures it with these, and so do Pedro's tuners
 * (pedro/Constants), which run without the Robot. Odometry Calibration keeps its own encoder directions.
 */
public final class PinpointSettings {
  private PinpointSettings() {}

  // CAD, to the middle of the four wheels, the centre ShooterGeometry uses. The old (0, -105) made
  // spinning in place read as 0.07 m/s forward and 0.10 m/s left per rad/s (fits 0.85-0.99 over four runs),
  // which worked back to offsets of about (-65, 0): the hooded shot's lead swung with every turn (5 Nov, 084206)
  public static final double X_OFFSET_MM = -71.5;
  public static final double Y_OFFSET_MM = 0.1;
  public static final GoBildaPinpointDriver.GoBildaOdometryPods POD =
      GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
  // X (forward) pod counts backward: TeleOp log 3 Nov 22:22, wheels driving the robot backward moved
  // the pose forward (30 of 31 samples). Y (strafe) counted backward too: pushing the robot right raised Y (4 Nov).
  public static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
      GoBildaPinpointDriver.EncoderDirection.REVERSED;
  public static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
      GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
