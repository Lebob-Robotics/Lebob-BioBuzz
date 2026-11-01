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

  /**
   * Turns a point seen by this camera into robot terms. The inputs are the SDK's ftcPose axes: right,
   * forward along the lens and up the picture. Returns {forward, left, up}, from the robot centre and
   * the floor, so the up value is the point's height.
   */
  public double[] toRobotFrame(double rightM, double lensForwardM, double pictureUpM) {
    double cosPitch = Math.cos(pitchRad); // Undo the camera's tilt
    double sinPitch = Math.sin(pitchRad);
    double levelForwardM = lensForwardM * cosPitch - pictureUpM * sinPitch;
    double levelUpM = lensForwardM * sinPitch + pictureUpM * cosPitch;
    double levelLeftM = -rightM;

    double cosYaw = Math.cos(yawRad); // Turn by the camera's yaw, then add where it is mounted
    double sinYaw = Math.sin(yawRad);
    return new double[] {
        forwardM + levelForwardM * cosYaw - levelLeftM * sinYaw,
        leftM + levelForwardM * sinYaw + levelLeftM * cosYaw,
        heightM + levelUpM};
  }
}
