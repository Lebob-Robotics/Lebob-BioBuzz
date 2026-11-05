package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
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
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.CameraMount;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.ClusterFromTags;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.TagObservation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.opencv.core.Mat;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
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
  /**
   * Saves a frame from each camera this often (0 never), to /sdcard/VisionPortal-cam{L,R}{0-9}.png, the
   * names cycling so a PC can pull them and see what the robot sees.
   */
  public static double SAVE_FRAME_EVERY_S = 5;
  private long lastFrameSaveNanos;
  private int frameSaves;
  public static float DECIMATION = 2; // Halves the detection work; 3.25-inch tags still read at 2+ m. 1 for range
  /**
   * Also run a single-tag detector on each camera, so every Hive tag in view (red and blue) reaches the
   * odometry correction. The cluster detector only reports whole Cells, which the correction can't use.
   * Costs a second detection pass per frame; read at INIT.
   */
  public static boolean SINGLE_TAGS_FOR_POSE = true;
  /**
   * Run only the single-tag detector and work the Cells out from its tags (ClusterFromTags), instead of
   * a second full detection pass for the SDK clusters. Two passes dropped the camera to 1-2 FPS with
   * frames a second old. Read at INIT.
   */
  public static boolean CLUSTERS_FROM_TAGS = true;
  private boolean clustersFromTags;

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
  private final AprilTagProcessor[] singleTagProcessors = new AprilTagProcessor[2]; // Null when not running
  private final VisionPortal[] portals = new VisionPortal[2];
  private final VisionInputs[] inputs = {new VisionInputs(), new VisionInputs()};
  private final String[] cameraStatus = {"not opened", "not opened"}; // Written by the camera worker too
  private volatile int[] appliedExposure = {-1, -1};
  private volatile int[] appliedGain = {-1, -1};
  private final float[] appliedDecimation = {Float.NaN, Float.NaN};
  /**
   * A camera that says it is streaming but gives no frames this long is stopped and started again. On
   * 4 Nov camera R hit a USB error starting its stream and sat at 0 FPS for a whole match.
   */
  public static double RESTART_AFTER_NO_FRAMES_S = 2.0;
  private final long[] noFramesSinceNanos = {Long.MIN_VALUE, Long.MIN_VALUE};
  private final int[] restarts = new int[2];
  /**
   * Restarts and camera controls run here, never on the robot loop: when the camera dropped off USB six times
   * in 3 s, stopStreaming() never returned and the loop froze with the flywheel on for 4 minutes (5 Nov, 080143).
   */
  private final ExecutorService cameraWorker = Executors.newSingleThreadExecutor(r -> {
    Thread t = new Thread(r, "BioBuzzCameraWorker");
    t.setDaemon(true);
    return t;
  });
  private final long[] streamingSinceNanos = {Long.MIN_VALUE, Long.MIN_VALUE};
  private final boolean[] workerBusy = new boolean[2]; // Set by the loop, cleared by the worker; guarded by this
  private final FrameClock[] frameClocks = {new FrameClock(), new FrameClock()};

  /**
   * Notes when each frame arrives. A frozen stream kept reporting its last FPS (9.2), so the no-frames restart
   * never ran and the robot drove 10 s blind, then searched with a dead camera (5 Nov, 082057).
   */
  private static final class FrameClock implements VisionProcessor {
    volatile long lastFrameNanos = System.nanoTime();

    @Override public void init(int width, int height, CameraCalibration calibration) {}

    @Override public Object processFrame(Mat frame, long captureTimeNanos) {
      lastFrameNanos = System.nanoTime();
      return null;
    }

    @Override public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
        float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
  }

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

    AprilTagLibrary clusterLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
    // BIOBUZZ cluster members are 3.25-inch tags. Leave out cluster metadata so the SDK
    // returns each visible tag centre, rather than a virtual cluster reference point.
    AprilTagLibrary.Builder tags = new AprilTagLibrary.Builder().addTags(clusterLibrary);
    for (int id = 30; id <= 45; id++) {
      tags.addTag(id, "BIOBUZZ " + id, 3.25, DistanceUnit.INCH);
    }
    AprilTagLibrary singleLibrary = tags.build();
    clustersFromTags = !individualTags && CLUSTERS_FROM_TAGS;
    AprilTagLibrary library = individualTags || clustersFromTags ? singleLibrary : clusterLibrary;
    AprilTagLibrary extraLibrary = !individualTags && !clustersFromTags && SINGLE_TAGS_FOR_POSE
        ? singleLibrary : null;
    setUpCamera(hardwareMap, Side.LEFT, "CameraL", LENS_LEFT, viewIds == null ? -1 : viewIds[0], library, extraLibrary);
    setUpCamera(hardwareMap, Side.RIGHT, "CameraR", LENS_RIGHT, viewIds == null ? -1 : viewIds[1], library, extraLibrary);
  }

  private static AprilTagProcessor buildProcessor(AprilTagLibrary library, double[] lens) {
    return new AprilTagProcessor.Builder()
        .setTagLibrary(library)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setLensIntrinsics(lens[0], lens[1], lens[2], lens[3])
        .build();
  }

  private void setUpCamera(HardwareMap hardwareMap, Side side, String name, double[] lens, int viewId,
      AprilTagLibrary library, AprilTagLibrary extraLibrary) {
    try {
      AprilTagProcessor processor = buildProcessor(library, lens);
      AprilTagProcessor singleTagProcessor = extraLibrary == null ? null : buildProcessor(extraLibrary, lens);

      VisionPortal.Builder builder = new VisionPortal.Builder()
          .setCamera(hardwareMap.get(WebcamName.class, name))
          .setCameraResolution(RESOLUTION)
          // Compressed, so two 720p streams fit through the hub's USB
          .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
          .addProcessor(processor)
          .addProcessor(frameClocks[side.ordinal()]);
      if (singleTagProcessor != null) {
        builder.addProcessor(singleTagProcessor);
      }
      if (viewId >= 0) {
        builder.setLiveViewContainerId(viewId);
      }

      portals[side.ordinal()] = builder.build();
      processors[side.ordinal()] = processor;
      singleTagProcessors[side.ordinal()] = singleTagProcessor;
      cameraStatus[side.ordinal()] = "opening";
    } catch (RuntimeException e) {
      cameraStatus[side.ordinal()] = e.toString();
      RobotLog.ee("BioBuzzVision", e, "Failed to open %s", name);
      portals[side.ordinal()] = null;
      processors[side.ordinal()] = null;
      singleTagProcessors[side.ordinal()] = null;
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
        in.capture(processors[side.ordinal()], singleTagProcessors[side.ordinal()], portals[side.ordinal()]);
        if (clustersFromTags) {
          in.addClusters(ClusterFromTags.build(in.tagIds, in.x, in.y, in.z, mountOf(side)));
        }
      }
      String prefix = side == Side.LEFT ? "Vision/Left" : "Vision/Right";
      Logger.processInputs(prefix, in);
      Logger.recordOutput(prefix + "/Status", cameraStatus[side.ordinal()]);
      Logger.recordOutput(prefix + "/Restarts", restarts[side.ordinal()]);
      Logger.recordOutput(prefix + "/LastFrameAgeS", (System.nanoTime() - frameClocks[side.ordinal()].lastFrameNanos) * 1e-9);
    }
    long now = System.nanoTime();
    if (!Logger.isReplay() && SAVE_FRAME_EVERY_S > 0 && (now - lastFrameSaveNanos) / 1e9 >= SAVE_FRAME_EVERY_S) {
      lastFrameSaveNanos = now;
      for (Side side : Side.values()) {
        VisionPortal portal = portals[side.ordinal()];
        if (portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
          portal.saveNextFrameRaw("cam" + (side == Side.LEFT ? "L" : "R") + (frameSaves % 10));
        }
      }
      frameSaves++;
    }
  }

  /** Apply settings only after streaming starts; never block the robot loop waiting for a camera. */
  private void applyCameraControls(Side side) {
    int i = side.ordinal();
    VisionPortal portal = portals[i];
    if (portal == null) {
      return;
    }
    synchronized (this) {
      if (workerBusy[i]) {
        streamingSinceNanos[i] = Long.MIN_VALUE;
        return;
      }
    }
    VisionPortal.CameraState state = portal.getCameraState();
    if (state != VisionPortal.CameraState.STREAMING) {
      appliedExposure[i] = -1;
      appliedGain[i] = -1;
      noFramesSinceNanos[i] = Long.MIN_VALUE;
      streamingSinceNanos[i] = Long.MIN_VALUE;
      cameraStatus[i] = state.name() + (restarts[i] > 0 ? " (restarts " + restarts[i] + ")" : "");
      return;
    }
    if (streamingSinceNanos[i] == Long.MIN_VALUE) {
      streamingSinceNanos[i] = System.nanoTime();
    }
    if (restartIfNoFrames(i, portal)) {
      return;
    }
    if (appliedDecimation[i] != DECIMATION) {
      processors[i].setDecimation(Math.max(1, DECIMATION));
      if (singleTagProcessors[i] != null) {
        singleTagProcessors[i].setDecimation(Math.max(1, DECIMATION));
      }
      appliedDecimation[i] = DECIMATION;
    }
    if (appliedExposure[i] == EXPOSURE_MS && appliedGain[i] == GAIN) {
      return;
    }
    final int exposureWanted = EXPOSURE_MS;
    final int gainWanted = GAIN;
    runOnWorker(i, () -> setControls(i, side, portal, exposureWanted, gainWanted));
  }

  private void setControls(int i, Side side, VisionPortal portal, int exposureWanted, int gainWanted) {
    try {
      ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
      GainControl gain = portal.getCameraControl(GainControl.class);
      boolean ok = exposure.isModeSupported(ExposureControl.Mode.Manual)
          && exposure.setMode(ExposureControl.Mode.Manual);
      long exposureMs = Math.max(exposure.getMinExposure(TimeUnit.MILLISECONDS),
          Math.min(exposure.getMaxExposure(TimeUnit.MILLISECONDS), exposureWanted));
      ok = ok && exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
      int gainValue = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), gainWanted));
      ok = gain.setGain(gainValue) && ok;
      cameraStatus[i] = (ok ? "streaming" : "controls partly unsupported")
          + ": exposure " + exposure.getExposure(TimeUnit.MILLISECONDS) + " ms, gain " + gain.getGain();
    } catch (RuntimeException e) {
      cameraStatus[i] = "streaming; controls: " + e.toString();
      RobotLog.ee("BioBuzzVision", e, "Camera controls failed for %s", side);
    }
    appliedExposure[i] = exposureWanted;
    appliedGain[i] = gainWanted;
  }

  /** Runs a camera call on the worker unless one for this camera is still going (it may never return). */
  private void runOnWorker(int i, Runnable job) {
    synchronized (this) {
      if (workerBusy[i]) {
        return;
      }
      workerBusy[i] = true;
    }
    cameraWorker.execute(() -> {
      try {
        job.run();
      } catch (RuntimeException e) {
        RobotLog.ee("BioBuzzVision", e, "Camera %d call failed", i);
      } finally {
        synchronized (VisionSubsystem.this) {
          workerBusy[i] = false;
        }
      }
    });
  }

  /** Stop, wait for the device, start again. On the worker. */
  private void restart(int i, VisionPortal portal) {
    portal.stopStreaming();
    long deadline = System.nanoTime() + 3_000_000_000L;
    while (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY && System.nanoTime() < deadline) {
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return;
      }
    }
    portal.resumeStreaming();
  }

  /** Seconds since any camera last gave a frame; 0 in replay or with no camera opened. */
  public double newestFrameAgeS() {
    double age = Double.POSITIVE_INFINITY;
    for (int i = 0; i < portals.length; i++) {
      if (portals[i] != null) {
        age = Math.min(age, (System.nanoTime() - frameClocks[i].lastFrameNanos) * 1e-9);
      }
    }
    return Double.isInfinite(age) ? 0 : age;
  }

  /** Stops a streaming camera that gives no frames; the next loops start it again. True if it did. */
  private boolean restartIfNoFrames(int i, VisionPortal portal) {
    long now = System.nanoTime();
    long expectedFrameSince = Math.max(frameClocks[i].lastFrameNanos, streamingSinceNanos[i]); // Grace after a (re)start
    boolean framesArriving = (now - expectedFrameSince) * 1e-9 < RESTART_AFTER_NO_FRAMES_S / 2;
    if (portal.getFps() >= 0.5f && framesArriving) {
      noFramesSinceNanos[i] = Long.MIN_VALUE;
      return false;
    }
    if (noFramesSinceNanos[i] == Long.MIN_VALUE) {
      noFramesSinceNanos[i] = now;
      return false;
    }
    if ((now - noFramesSinceNanos[i]) * 1e-9 < RESTART_AFTER_NO_FRAMES_S) {
      return false;
    }
    RobotLog.ww("BioBuzzVision", "Camera %d streaming with no frames; restarting", i);
    restarts[i]++;
    noFramesSinceNanos[i] = Long.MIN_VALUE;
    cameraStatus[i] = "no frames, restarting (" + restarts[i] + ")";
    appliedExposure[i] = -1;
    appliedGain[i] = -1;
    runOnWorker(i, () -> restart(i, portal));
    return true;
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
    cameraWorker.shutdownNow();
    for (int i = 0; i < portals.length; i++) {
      if (portals[i] != null) {
        portals[i].close();
        portals[i] = null;
        processors[i] = null;
        singleTagProcessors[i] = null;
      }
    }
  }
}
