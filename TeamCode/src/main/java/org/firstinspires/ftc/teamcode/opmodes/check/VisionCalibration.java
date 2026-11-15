package org.firstinspires.ftc.teamcode.opmodes.check;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.field.Units;
import org.firstinspires.ftc.teamcode.subsystems.VisionInputs;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.CameraMount;
import org.firstinspires.ftc.teamcode.vision.CameraMounts;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;
import org.firstinspires.ftc.teamcode.opmodes.Main;

/**
 * Measures individual tags, and in INIT sweeps the cameras through {@link #SWEEP} so one log compares the exposure
 * settings on the same tags (tools/vision/exposure_report.py). No drive, shooter, hood or feed subsystems are
 * constructed, so nothing can move. START stops the sweep at the field preset.
 */
@TeleOp(name = "Vision Calibration", group = "Tuning")
public class VisionCalibration extends PsiKitIterativeOpMode {
  /** {exposure ms, gain} pairs, around the comp venue's 2 ms / gain 15-20. */
  static final int[][] SWEEP = {
      {2, 20}, {1, 20}, {1, 30}, {1, 40}, {2, 10}, {2, 15}, {2, 25}, {2, 30}, {3, 15}, {3, 20}, {3, 30},
  };
  /** Per setting; the report drops the first 0.5 s after a change while the camera settles. */
  static final double STEP_S = 4.0;

  private VisionSubsystem vision;
  private long sweepStartNanos;
  private boolean sweeping;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }

  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Vision Calibration");
  }

  @Override protected void onPsiKitInit() {
    vision = new VisionSubsystem(hardwareMap, true);
    sweepStartNanos = System.nanoTime();
    sweeping = true;
  }

  /** The sweep step at {@code elapsedS} into it, cycling. */
  static int stepAt(double elapsedS) {
    return (int) (Math.max(0, elapsedS) / STEP_S) % SWEEP.length;
  }

  @Override protected void onPsiKitInitLoop() {
    double elapsedS = (System.nanoTime() - sweepStartNanos) * 1e-9;
    int step = stepAt(elapsedS);
    VisionSubsystem.overrideExposureMs = SWEEP[step][0];
    VisionSubsystem.overrideGain = SWEEP[step][1];
    Logger.recordOutput("Vision/Calibration/SweepStep", step);
    Logger.recordOutput("Vision/Calibration/SweepRounds", (int) (elapsedS / (STEP_S * SWEEP.length)));
    telemetry.addLine("EXPOSURE SWEEP: hold still facing Hive tags 1-2 m away. 2 rounds (~90 s), then Stop.");
    telemetry.addData("Setting", "%d/%d: %d ms, gain %d   round %d", step + 1, SWEEP.length, SWEEP[step][0],
        SWEEP[step][1], (int) (elapsedS / (STEP_S * SWEEP.length)) + 1);
    readVision();
  }

  @Override protected void onPsiKitStart() {
    sweeping = false;
    VisionSubsystem.overrideExposureMs = -1;
    VisionSubsystem.overrideGain = -1;
  }

  @Override protected void onPsiKitLoop() { readVision(); }

  private void readVision() {
    vision.periodic();
    telemetry.addLine("Vision only. Measure horizontally from below lens to below individual tag centre.");
    telemetry.addLine("Floor distance uses configured camera pitch; verify the physical mount.");
    for (VisionSubsystem.Side side : VisionSubsystem.Side.values()) {
      VisionInputs in = vision.getInputs(side);
      CameraMount mount = side == VisionSubsystem.Side.LEFT
          ? CameraMounts.LEFT_CAMERA : CameraMounts.RIGHT_CAMERA;
      double[] horizontalM = new double[in.tagIds.length];
      double[] slantM = new double[in.tagIds.length];
      for (int i = 0; i < in.tagIds.length; i++) {
        double x = in.x[i] * Units.M_PER_IN, y = in.y[i] * Units.M_PER_IN, z = in.z[i] * Units.M_PER_IN;
        horizontalM[i] = mount.horizontalDistanceFromLens(x, y, z);
        slantM[i] = Math.sqrt(x * x + y * y + z * z);
        telemetry.addData(side + " tag " + in.tagIds[i], "floor %.3f m, slant %.3f m, margin %.0f",
            horizontalM[i], slantM[i], in.decisionMargin[i]);
      }
      String prefix = "Vision/Calibration/" + side;
      Logger.recordOutput(prefix + "/LensHorizontalM", horizontalM);
      Logger.recordOutput(prefix + "/LensSlantM", slantM);
      telemetry.addData(side + " camera", vision.describe(side));
    }
    telemetry.update();
  }

  @Override protected void onPsiKitStop() {
    VisionSubsystem.overrideExposureMs = -1;
    VisionSubsystem.overrideGain = -1;
    if (vision != null) { vision.stop(); }
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
