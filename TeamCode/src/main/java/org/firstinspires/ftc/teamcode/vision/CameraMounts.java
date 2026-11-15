package org.firstinspires.ftc.teamcode.vision;

/**
 * The cameras' lens centres from the robot centre (robot CAD): both face backward and 35 degrees out, tilted up 41.1
 * degrees. Measured rather than tuned, so not on Panels.
 */
public final class CameraMounts {
  private CameraMounts() {}

  // Both cameras are mounted upside down: roll must be undone before pitch and yaw.
  // The replacement left camera's yaw and pitch are fitted to the right camera's tag positions, not CAD
  // (6 Nov calibration drive, 824 sightings both cameras made of the same tags).
  public static final CameraMount LEFT_CAMERA = new CameraMount(
      -0.1624, 0.0377, 0.2093, Math.PI - Math.toRadians(37), Math.toRadians(39.85), Math.PI);
  public static final CameraMount RIGHT_CAMERA = new CameraMount(
      -0.1457, -0.0625, 0.2093, -(Math.PI - Math.toRadians(35)), Math.toRadians(41.09), Math.PI);
}
