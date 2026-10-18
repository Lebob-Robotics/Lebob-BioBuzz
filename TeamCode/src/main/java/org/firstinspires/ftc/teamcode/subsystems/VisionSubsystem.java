package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CameraMount;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Wraps two webcams, each running an AprilTag processor, and reports their
 * detections. The two live views share the Robot Controller preview.
 *
 * <p>A camera that is missing or fails to start is left unavailable instead of
 * crashing init.
 */
public class VisionSubsystem extends SubsystemBase {
  public enum Side {
    LEFT, RIGHT
  }

  private final AprilTagProcessor[] processors = new AprilTagProcessor[2];
  private final VisionPortal[] portals = new VisionPortal[2];
  private final VisionInputs[] inputs = {new VisionInputs(), new VisionInputs()};

  public VisionSubsystem(HardwareMap hardwareMap) {
    // Replay has no cameras: the detections come from the log instead.
    if (Logger.isReplay()) {
      return;
    }

    int[] viewIds = null;
    try {
      viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    } catch (RuntimeException e) {
      // Fall back to portals without a shared live view.
    }

    setUpCamera(hardwareMap, Side.LEFT, "CameraL", viewIds == null ? -1 : viewIds[0]);
    setUpCamera(hardwareMap, Side.RIGHT, "CameraR", viewIds == null ? -1 : viewIds[1]);
  }

  private void setUpCamera(HardwareMap hardwareMap, Side side, String name, int viewId) {
    try {
      AprilTagProcessor processor = new AprilTagProcessor.Builder()
          .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
          .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
          .build();

      VisionPortal.Builder builder = new VisionPortal.Builder()
          .setCamera(hardwareMap.get(WebcamName.class, name))
          .addProcessor(processor);
      if (viewId >= 0) {
        builder.setLiveViewContainerId(viewId);
      }

      portals[side.ordinal()] = builder.build();
      processors[side.ordinal()] = processor;
    } catch (RuntimeException e) {
      portals[side.ordinal()] = null;
      processors[side.ordinal()] = null;
    }
  }

  /**
   * Reads each camera (live only) and logs it. Everything else in this class
   * reads from the logged inputs, so replay sees what the robot saw.
   */
  @Override
  public void periodic() {
    for (Side side : Side.values()) {
      VisionInputs in = inputs[side.ordinal()];
      if (!Logger.isReplay()) {
        in.capture(processors[side.ordinal()], portals[side.ordinal()]);
      }
      Logger.processInputs(side == Side.LEFT ? "Vision/Left" : "Vision/Right", in);
    }
  }

  private static final double INCHES_TO_METRES = 0.0254; // The processor is set to inches

  private final long[] newestFrameHandedOutNanos = {Long.MIN_VALUE, Long.MIN_VALUE}; // Per camera

  /** System.nanoTime() of the latest capture. Robot uses it as the loop time, so replay matches. */
  public long getCaptureNanos() {
    return Math.max(inputs[0].captureNanos, inputs[1].captureNanos);
  }

  /**
   * Cell cluster sightings from both cameras, from frames not handed out before. Call once per loop.
   * Reads only the logged inputs, so replay behaves like the live run.
   */
  public List<CellSighting> takeNewCellSightings() {
    List<CellSighting> sightings = new ArrayList<>();
    for (Side side : Side.values()) {
      VisionInputs camera = inputs[side.ordinal()];
      CameraMount mount = side == Side.LEFT
          ? ShootOnTheMoveConstants.LEFT_CAMERA
          : ShootOnTheMoveConstants.RIGHT_CAMERA;
      int detectionCount = camera.tagIds.length;
      boolean oldLog = camera.clusterNames.length != detectionCount // Logged before every array existed
          || camera.frameNanos.length != detectionCount;
      if (oldLog) {
        continue;
      }
      long previousNewestNanos = newestFrameHandedOutNanos[side.ordinal()];
      long newestNanos = previousNewestNanos;
      for (int i = 0; i < detectionCount; i++) {
        boolean isCluster = camera.tagIds[i] == VisionInputs.CLUSTER_ID;
        boolean isNewFrame = camera.frameNanos[i] > previousNewestNanos;
        if (isCluster && isNewFrame && !Double.isNaN(camera.x[i])) {
          sightings.add(new CellSighting(camera.clusterNames[i], mount, camera.frameNanos[i],
              camera.x[i] * INCHES_TO_METRES, camera.y[i] * INCHES_TO_METRES, camera.z[i] * INCHES_TO_METRES));
        }
        newestNanos = Math.max(newestNanos, camera.frameNanos[i]);
      }
      newestFrameHandedOutNanos[side.ordinal()] = newestNanos;
    }
    return sightings;
  }

  public boolean isAvailable(Side side) {
    return inputs[side.ordinal()].available;
  }

  /** The latest logged inputs for one camera. */
  public VisionInputs getInputs(Side side) {
    return inputs[side.ordinal()];
  }

  /** Short telemetry string, e.g. "id 20 @ 42.1 in, id 24 @ 60.0 in". */
  public String describe(Side side) {
    return inputs[side.ordinal()].describe();
  }

  /** Closes both portals. Call from the OpMode's stop(). */
  public void close() {
    for (int i = 0; i < portals.length; i++) {
      if (portals[i] != null) {
        portals[i].close();
        portals[i] = null;
        processors[i] = null;
      }
    }
  }
}
