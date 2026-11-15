package org.firstinspires.ftc.teamcode.robot;

import java.util.ArrayList;
import java.util.List;

/** The lines the SDK's RobotLog wrote through the test stand-in for android.util.Log. */
public final class AndroidLogLines {
  private static final List<String> LINES = new ArrayList<>();

  private AndroidLogLines() {}

  public static synchronized void add(String line) {
    LINES.add(line);
  }

  /** Every line since the last call. */
  static synchronized List<String> drain() {
    List<String> lines = new ArrayList<>(LINES);
    LINES.clear();
    return lines;
  }
}
