package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class CameraMountTest {
  @Test public void removesCameraPitchBeforeMeasuringAlongFloor() {
    double pitch = Math.toRadians(41.09);
    double horizontal = 0.72, height = 1.0;
    double lensForward = horizontal * Math.cos(pitch) + height * Math.sin(pitch);
    double pictureUp = -horizontal * Math.sin(pitch) + height * Math.cos(pitch);
    CameraMount camera = new CameraMount(-0.1457, -0.0625, 0.2093, -2.53, pitch);
    assertEquals(horizontal, camera.horizontalDistanceFromLens(0, lensForward, pictureUp), 1e-9);
  }

  @Test public void includesLateralSeparationButExcludesRobotCentreOffset() {
    CameraMount camera = new CameraMount(1, 2, 3, 1.2, 0);
    assertEquals(0.5, camera.horizontalDistanceFromLens(0.3, 0.4, 1), 1e-9);
  }

  @Test public void upsideDownCameraRecoversBothHeightAndLateralPosition() {
    double pitch = Math.toRadians(41.09);
    double forward = 0.72, left = 0.1, up = 1.0;
    double lensForward = forward * Math.cos(pitch) + up * Math.sin(pitch);
    double pictureUp = -(-forward * Math.sin(pitch) + up * Math.cos(pitch));
    CameraMount camera = new CameraMount(0, 0, 0, 0, pitch, Math.PI);
    double[] robot = camera.toRobotFrame(left, lensForward, pictureUp);
    assertEquals(forward, robot[0], 1e-9);
    assertEquals(left, robot[1], 1e-9);
    assertEquals(up, robot[2], 1e-9);
    assertEquals(Math.hypot(forward, left),
        camera.horizontalDistanceFromLens(left, lensForward, pictureUp), 1e-9);
  }

  @Test public void recordedTag31IsAboutSeventyCentimetresAwayWithRollCorrected() {
    CameraMount camera = new CameraMount(0, 0, 0.2093, 0, Math.toRadians(41.09), Math.PI);
    double x = 3.98 * 0.0254, y = 48.17 * 0.0254, z = -13.54 * 0.0254;
    assertEquals(0.70, camera.horizontalDistanceFromLens(x, y, z), 0.01);
    assertEquals(1.27, camera.toRobotFrame(x, y, z)[2], 0.02);
  }
}
