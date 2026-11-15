package org.firstinspires.ftc.teamcode.tuning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import org.psilynx.psikit.core.Logger;

/** The tuning OpModes' CSVs on the Hub: a header line when the file is new, then one row a line. */
public final class TuningCsv {
  /** Null on the robot. Tests point it at a folder, where the CSVs go under their own names, replay or not. */
  static File directory;

  private TuningCsv() {}

  /** The file for a Hub path such as /sdcard/FIRST/ShotTuning_v2.csv. */
  public static File file(String hubPath) {
    return directory == null ? new File(hubPath) : new File(directory, new File(hubPath).getName());
  }

  /** Adds a row, after the header if the file is new. False, writing nothing, in replay: it is not the robot's run. */
  public static boolean append(String hubPath, String header, String row) throws IOException {
    if (Logger.isReplay() && directory == null) {
      return false;
    }
    File file = file(hubPath);
    boolean isNew = !file.exists();
    try (FileWriter out = new FileWriter(file, true)) {
      if (isNew) {
        out.write(header + "\n");
      }
      out.write(row + "\n");
    }
    return true;
  }
}
