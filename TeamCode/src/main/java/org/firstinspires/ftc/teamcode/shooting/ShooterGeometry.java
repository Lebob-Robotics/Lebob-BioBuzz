package org.firstinspires.ftc.teamcode.shooting;

/**
 * Where the shooter sits and points on the robot, from the robot CAD (Onshape export of 25 Oct 2026, FTC2026/files), and the hood's
 * top stop. Positions are from the robot centre, the middle of the four mecanum wheels, the same point as the Pinpoint
 * offsets (PinpointSettings). Metres and radians unless the name says otherwise. Measured rather than tuned, so not on
 * Panels; the ones tests change aren't final.
 */
public final class ShooterGeometry {
  private ShooterGeometry() {}

  public static final double SHOOTER_DIRECTION_RAD = Math.PI; // Relative to robot forward. PI fires out of the back
  public static double SHOOTER_FORWARD_M = -0.0066; // Flywheel axle. The sim works out the exit point from the hood angle
  public static double SHOOTER_LEFT_M = 0.0007;
  public static final double BACKWARD_DIRECTION_RAD = 0; // Relative to robot forward

  // Launch angle on the hood's upper hard stop (Hood Tune RANGE). The hood's soft limit and the shots both come
  // from it, so a shot flatter than the hood can reach is no shot rather than one fired at the stop's angle.
  // 54.3: the second hard stop, 5 Nov (HoodTune 083632, lift 44.7); 58.5 the first; 51 before stops
  public static double HOOD_TOP_STOP_DEG = 54.3;
}
