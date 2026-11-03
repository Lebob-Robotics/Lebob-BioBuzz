package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.RobotLog;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CameraMount;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.TagObservation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Wraps two webcams, each running an AprilTag processor, and reports their
 * detections. The two live views share the Robot Controller preview.
 *
 * <p>A camera that is missing or fails to start is left unavailable instead of
 * crashing init.
 */
@Configurable
public class VisionSubsystem extends SubsystemBase {
  /** Live camera controls for tuning tag contrast. Exposure is in milliseconds. */
  public static int EXPOSURE_MS = 6;
  public static int GAIN = 60;
  public static float DECIMATION = 1;

  public enum Side {
    LEFT, RIGHT
  }

  // ---- Cameras ---------------------------------------------------------
  /** Both cameras run at this size. The lens calibrations below are only valid at it. */
  private static final Size RESOLUTION = new Size(1280, 720);
  /**
   * Lens calibration {fx, fy, cx, cy} in pixels at RESOLUTION, per physical camera, from
   * tools/camera-calibration (results/*.json, 1 Nov). Recalibrate if a lens is refocused or swapped.
   */
  private static final double[] LENS_LEFT = {892.442, 892.360, 632.589, 355.601};
  private static final double[] LENS_RIGHT = {885.382, 885.080, 677.418, 365.001};

  private final AprilTagProcessor[] processors = new AprilTagProcessor[2];
  private final VisionPortal[] portals = new VisionPortal[2];
  private final VisionInputs[] inputs = {new VisionInputs(), new VisionInputs()};
  private final String[] cameraStatus = {"not opened", "not opened"};
  private final int[] appliedExposure = {-1, -1};
  private final int[] appliedGain = {-1, -1};
  private final float[] appliedDecimation = {Float.NaN, Float.NaN};

  public VisionSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap, false);
  }

  /** Individual-tag mode is for the vision-only calibration OpMode; normal shooting uses clusters. */
  public VisionSubsystem(HardwareMap hardwareMap, boolean individualTags) {
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

    AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();
    if (individualTags) {
      // BIOBUZZ cluster members are 3.25-inch tags. Leave out cluster metadata so the SDK
      // returns each visible tag centre, rather than a virtual cluster reference point.
      AprilTagLibrary.Builder tags = new AprilTagLibrary.Builder().addTags(library);
      for (int id = 30; id <= 45; id++) {
        tags.addTag(id, "BIOBUZZ " + id, 3.25, DistanceUnit.INCH);
      }
      library = tags.build();
    }
    setUpCamera(hardwareMap, Side.LEFT, "CameraL", LENS_LEFT, viewIds == null ? -1 : viewIds[0], library);
    setUpCamera(hardwareMap, Side.RIGHT, "CameraR", LENS_RIGHT, viewIds == null ? -1 : viewIds[1], library);
  }

  private void setUpCamera(HardwareMap hardwareMap, Side side, String name, double[] lens, int viewId, AprilTagLibrary library) {
    try {
      AprilTagProcessor processor = new AprilTagProcessor.Builder()
          .setTagLibrary(library)
          .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
          .setLensIntrinsics(lens[0], lens[1], lens[2], lens[3])
          .build();

      VisionPortal.Builder builder = new VisionPortal.Builder()
          .setCamera(hardwareMap.get(WebcamName.class, name))
          .setCameraResolution(RESOLUTION)
          // Compressed, so two 720p streams fit through the hub's USB
          .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
          .addProcessor(processor);
      if (viewId >= 0) {
        builder.setLiveViewContainerId(viewId);
      }

      portals[side.ordinal()] = builder.build();
      processors[side.ordinal()] = processor;
      cameraStatus[side.ordinal()] = "opening";
    } catch (RuntimeException e) {
      cameraStatus[side.ordinal()] = e.toString();
      RobotLog.ee("BioBuzzVision", e, "Failed to open %s", name);
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
        applyCameraControls(side);
        in.capture(processors[side.ordinal()], portals[side.ordinal()]);
      }
      String prefix = side == Side.LEFT ? "Vision/Left" : "Vision/Right";
      Logger.processInputs(prefix, in);
      Logger.recordOutput(prefix + "/Status", cameraStatus[side.ordinal()]);
    }
  }

  /** Apply settings only after streaming starts; never block the robot loop waiting for a camera. */
  private void applyCameraControls(Side side) {
    int i = side.ordinal();
    VisionPortal portal = portals[i];
    if (portal == null) {
      return;
    }
    if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
      appliedExposure[i] = -1;
      appliedGain[i] = -1;
      cameraStatus[i] = portal.getCameraState().name();
      return;
    }
    if (appliedDecimation[i] != DECIMATION) {
      processors[i].setDecimation(Math.max(1, DECIMATION));
      appliedDecimation[i] = DECIMATION;
    }
    if (appliedExposure[i] == EXPOSURE_MS && appliedGain[i] == GAIN) {
      return;
    }
    try {
      ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
      GainControl gain = portal.getCameraControl(GainControl.class);
      boolean ok = exposure.isModeSupported(ExposureControl.Mode.Manual)
          && exposure.setMode(ExposureControl.Mode.Manual);
      long exposureMs = Math.max(exposure.getMinExposure(TimeUnit.MILLISECONDS),
          Math.min(exposure.getMaxExposure(TimeUnit.MILLISECONDS), EXPOSURE_MS));
      ok = ok && exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
      int gainValue = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN));
      ok = gain.setGain(gainValue) && ok;
      cameraStatus[i] = (ok ? "streaming" : "controls partly unsupported")
          + ": exposure " + exposure.getExposure(TimeUnit.MILLISECONDS) + " ms, gain " + gain.getGain();
    } catch (RuntimeException e) {
      cameraStatus[i] = "streaming; controls: " + e.toString();
      RobotLog.ee("BioBuzzVision", e, "Camera controls failed for %s", side);
    }
    appliedExposure[i] = EXPOSURE_MS;
    appliedGain[i] = GAIN;
  }

  private static final double INCHES_TO_METRES = 0.0254; // The processor is set to inches

  // Each consumer keeps its own mark per camera, so taking frames for one never starves the other.
  private final long[] newestCellFrameHandedOutNanos = {Long.MIN_VALUE, Long.MIN_VALUE};
  private final long[] newestTagFrameHandedOutNanos = {Long.MIN_VALUE, Long.MIN_VALUE};

  /** System.nanoTime() of the latest capture. Robot uses it as the loop time, so replay matches. */
  public long getCaptureNanos() {
    return Math.max(inputs[0].captureNanos, inputs[1].captureNanos);
  }

  private static CameraMount mountOf(Side side) {
    return side == Side.LEFT ? ShootOnTheMoveConstants.LEFT_CAMERA : ShootOnTheMoveConstants.RIGHT_CAMERA;
  }

  /**
   * Cell cluster sightings from both cameras, from frames not handed out before. Call once per loop.
   * Reads only the logged inputs, so replay behaves like the live run.
   */
  public List<CellSighting> takeNewCellSightings() {
    List<CellSighting> sightings = new ArrayList<>();
    for (Side side : Side.values()) {
      VisionInputs camera = inputs[side.ordinal()];
      CameraMount mount = mountOf(side);
      int detectionCount = camera.tagIds.length;
      boolean oldLog = camera.clusterNames.length != detectionCount // Logged before every array existed
          || camera.frameNanos.length != detectionCount;
      if (oldLog) {
        continue;
      }
      long previousNewestNanos = newestCellFrameHandedOutNanos[side.ordinal()];
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
      newestCellFrameHandedOutNanos[side.ordinal()] = newestNanos;
    }
    return sightings;
  }

  /**
   * Single AprilTag observations from both cameras, for the pose correction, from frames not handed
   * out before. Cluster detections are left out: those are the Cell sightings. Call once per loop.
   * Reads only the logged inputs, so replay behaves like the live run.
   */
  public List<TagObservation> takeNewTagObservations() {
    List<TagObservation> observations = new ArrayList<>();
    for (Side side : Side.values()) {
      VisionInputs camera = inputs[side.ordinal()];
      CameraMount mount = mountOf(side);
      int detectionCount = camera.tagIds.length;
      boolean oldLog = camera.frameNanos.length != detectionCount // Logged before every array existed
          || camera.x.length != detectionCount || camera.y.length != detectionCount
          || camera.z.length != detectionCount || camera.decisionMargin.length != detectionCount;
      if (oldLog) {
        continue;
      }
      long previousNewestNanos = newestTagFrameHandedOutNanos[side.ordinal()];
      long newestNanos = previousNewestNanos;
      for (int i = 0; i < detectionCount; i++) {
        boolean isSingleTag = camera.tagIds[i] != VisionInputs.CLUSTER_ID;
        boolean isNewFrame = camera.frameNanos[i] > previousNewestNanos;
        if (isSingleTag && isNewFrame) {
          observations.add(new TagObservation(camera.tagIds[i], mount,
              camera.x[i] * INCHES_TO_METRES, camera.y[i] * INCHES_TO_METRES, camera.z[i] * INCHES_TO_METRES,
              camera.decisionMargin[i], camera.frameNanos[i]));
        }
        newestNanos = Math.max(newestNanos, camera.frameNanos[i]);
      }
      newestTagFrameHandedOutNanos[side.ordinal()] = newestNanos;
    }
    return observations;
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
    return inputs[side.ordinal()].describe() + " | " + cameraStatus[side.ordinal()];
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
