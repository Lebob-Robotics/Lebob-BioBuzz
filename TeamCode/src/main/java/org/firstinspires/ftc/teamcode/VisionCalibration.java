package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionInputs;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.CameraMount;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/** Measures individual tags in INIT. No drive, shooter, hood or feed subsystems are constructed. */
@TeleOp(name = "Vision Calibration", group = "Tuning")
public class VisionCalibration extends PsiKitIterativeOpMode {
  private VisionSubsystem vision;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }

  @Override protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("OpMode", "Vision Calibration");
  }

  @Override protected void onPsiKitInit() {
    vision = new VisionSubsystem(hardwareMap, true);
  }

  @Override protected void onPsiKitInitLoop() { readVision(); }
  @Override protected void onPsiKitStart() {}
  @Override protected void onPsiKitLoop() { readVision(); }

  private void readVision() {
    vision.periodic();
    telemetry.addLine("Vision only. Measure horizontally from below lens to below individual tag centre.");
    telemetry.addLine("Floor distance uses configured camera pitch; verify the physical mount.");
    for (VisionSubsystem.Side side : VisionSubsystem.Side.values()) {
      VisionInputs in = vision.getInputs(side);
      CameraMount mount = side == VisionSubsystem.Side.LEFT
          ? ShootOnTheMoveConstants.LEFT_CAMERA : ShootOnTheMoveConstants.RIGHT_CAMERA;
      double[] horizontalM = new double[in.tagIds.length];
      double[] slantM = new double[in.tagIds.length];
      for (int i = 0; i < in.tagIds.length; i++) {
        double x = in.x[i] * 0.0254, y = in.y[i] * 0.0254, z = in.z[i] * 0.0254;
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
    if (vision != null) { vision.close(); }
    com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
  }
}
