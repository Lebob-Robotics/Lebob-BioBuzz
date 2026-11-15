package org.firstinspires.ftc.teamcode.tuning;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

  /** Every line, the header first; null when there is no file yet. */
  public static List<String> readLines(String hubPath) throws IOException {
    File file = file(hubPath);
    if (!file.exists()) {
      return null;
    }
    List<String> lines = new ArrayList<>();
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) {
        lines.add(line);
      }
    }
    return lines;
  }

  /** Each column's index, by its name in the header. */
  public static Map<String, Integer> columns(String header) {
    Map<String, Integer> columns = new HashMap<>();
    String[] names = header.split(",");
    for (int i = 0; i < names.length; i++) {
      columns.put(names[i], i);
    }
    return columns;
  }

  /** The row's value in that column; empty when the header has no such column or the row stops short of it. */
  public static String field(String[] row, Map<String, Integer> columns, String name) {
    Integer i = columns.get(name);
    return i == null || i >= row.length ? "" : row[i];
  }
}
