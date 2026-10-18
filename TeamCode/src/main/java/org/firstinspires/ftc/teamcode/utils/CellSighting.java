package org.firstinspires.ftc.teamcode.utils;

/**
 * One camera sighting of a Cell's AprilTag cluster. The position is from the camera's point of view,
 * the same as the SDK's ftcPose.
 */
public final class CellSighting {
  public final String clusterName; // For example "RED SCORING"
  public final CameraMount camera;
  public final long frameTimeNanos; // System.nanoTime() when the frame was taken
  public final double rightM;
  public final double forwardM; // Along the lens
  public final double upM;

  public CellSighting(String clusterName, CameraMount camera, long frameTimeNanos,
      double rightM, double forwardM, double upM) {
    this.clusterName = clusterName;
    this.camera = camera;
    this.frameTimeNanos = frameTimeNanos;
    this.rightM = rightM;
    this.forwardM = forwardM;
    this.upM = upM;
  }
}
