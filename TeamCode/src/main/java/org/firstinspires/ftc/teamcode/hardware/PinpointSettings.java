package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * How the Pinpoint is set up: OdometrySubsystem configures it with these, and so do Pedro's tuners
 * (pedro/Constants), which run without the Robot. Odometry Calibration keeps its own encoder directions.
 */
public final class PinpointSettings {
  private PinpointSettings() {}

  // CAD, to the middle of the four wheels, the centre ShooterGeometry uses. Wrong offsets made spinning in
  // place read as driving, and the hooded shot's lead swung with every turn
  public static final double X_OFFSET_MM = -71.5;
  public static final double Y_OFFSET_MM = 0.1;
  public static final GoBildaPinpointDriver.GoBildaOdometryPods POD =
      GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
  // X (forward) pod counts backward: wheels driving the robot backward moved the pose forward. Y (strafe)
  // counted backward too: pushing the robot right raised Y.
  public static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
      GoBildaPinpointDriver.EncoderDirection.REVERSED;
  public static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
      GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
