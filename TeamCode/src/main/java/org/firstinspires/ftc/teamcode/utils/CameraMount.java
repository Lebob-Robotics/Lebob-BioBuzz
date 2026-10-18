package org.firstinspires.ftc.teamcode.utils;

/** Where a camera sits on the robot, measured from the robot centre (the Pinpoint tracking point). */
public final class CameraMount {
  public final double forwardM;
  public final double leftM;
  public final double heightM; // Above the floor
  public final double yawRad; // Turned left of robot forward
  public final double pitchRad; // Tilted up from level

  public CameraMount(double forwardM, double leftM, double heightM, double yawRad, double pitchRad) {
    this.forwardM = forwardM;
    this.leftM = leftM;
    this.heightM = heightM;
    this.yawRad = yawRad;
    this.pitchRad = pitchRad;
  }
}
