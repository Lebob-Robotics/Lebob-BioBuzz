package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.psilynx.psikit.core.Logger;

/**
 * One {@link SotmFieldTest} trial, run once with balls loaded, so a camera can show whether they score. Which
 * trial and alliance come from {@link #CONFIG}, a line "RED 7" or "BLUE 0" (alliance, then the trial's index), so
 * a PC can choose it between runs: adb shell run-as into the app's files folder, else adb push to CONFIG. Load the balls while no OpMode is running.
 */
@TeleOp(name = "SOTM Ball Test", group = "Tuning")
public class SotmBallTest extends SotmFieldTest {
  static final String CONFIG = "/sdcard/FIRST/sotm_ball_test.txt";

  /** The app's own copy, written with adb shell run-as. */
  static final String PRIVATE_CONFIG = "sotm_ball_test.txt";

  @Override protected void configureTest() {
    onlyTrial = 0;
    // adb writes /sdcard under the app's storage view, which at times still showed the app an empty file
    String line = readFirstLine(new File(hardwareMap.appContext.getFilesDir(), PRIVATE_CONFIG).getPath());
    Logger.recordOutput("FieldTest/BallConfigSource", line.isEmpty() ? CONFIG : PRIVATE_CONFIG);
    if (line.isEmpty()) {
      line = readFirstLine(CONFIG);
    }
    String[] words = line.split("\\s+");
    if (words.length >= 2) {
      testAlliance = "BLUE".equalsIgnoreCase(words[0]) ? Alliance.BLUE : Alliance.RED;
      try {
        onlyTrial = Math.max(0, Integer.parseInt(words[1]));
      } catch (NumberFormatException e) {
        onlyTrial = 0;
      }
    }
    Logger.recordOutput("FieldTest/BallConfig", line);
  }

  private static String readFirstLine(String path) {
    try (BufferedReader in = new BufferedReader(new FileReader(path))) {
      String read = in.readLine();
      return read == null ? "" : read.trim();
    } catch (IOException | RuntimeException e) {
      Logger.recordOutput("FieldTest/BallConfigError", e.toString());
      return "";
    }
  }
}
