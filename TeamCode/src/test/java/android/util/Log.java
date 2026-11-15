package android.util;

import org.firstinspires.ftc.teamcode.robot.AndroidLogLines;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Stands in for Android's Log in unit tests, whose stub throws: the SDK's RobotLog writes through it when the robot
 * trips the intake or shooter guards. Hands the lines to {@link AndroidLogLines} so a test can check them. Test code
 * can't call anything here that Android's Log lacks: javac may resolve this class from android.jar.
 */
public final class Log {
  public static final int VERBOSE = 2;
  public static final int DEBUG = 3;
  public static final int INFO = 4;
  public static final int WARN = 5;
  public static final int ERROR = 6;
  public static final int ASSERT = 7;

  private Log() {}

  public static int println(int priority, String tag, String msg) {
    AndroidLogLines.add(priority + " " + tag + ": " + msg);
    return msg == null ? 0 : msg.length();
  }

  public static boolean isLoggable(String tag, int level) {
    return true;
  }

  public static String getStackTraceString(Throwable tr) {
    if (tr == null) {
      return "";
    }
    StringWriter sw = new StringWriter();
    tr.printStackTrace(new PrintWriter(sw));
    return sw.toString();
  }

  public static int v(String tag, String msg) { return println(VERBOSE, tag, msg); }
  public static int v(String tag, String msg, Throwable tr) { return println(VERBOSE, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int d(String tag, String msg) { return println(DEBUG, tag, msg); }
  public static int d(String tag, String msg, Throwable tr) { return println(DEBUG, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int i(String tag, String msg) { return println(INFO, tag, msg); }
  public static int i(String tag, String msg, Throwable tr) { return println(INFO, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int w(String tag, String msg) { return println(WARN, tag, msg); }
  public static int w(String tag, String msg, Throwable tr) { return println(WARN, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int w(String tag, Throwable tr) { return println(WARN, tag, getStackTraceString(tr)); }
  public static int e(String tag, String msg) { return println(ERROR, tag, msg); }
  public static int e(String tag, String msg, Throwable tr) { return println(ERROR, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int wtf(String tag, String msg) { return println(ASSERT, tag, msg); }
  public static int wtf(String tag, String msg, Throwable tr) { return println(ASSERT, tag, msg + '\n' + getStackTraceString(tr)); }
  public static int wtf(String tag, Throwable tr) { return println(ASSERT, tag, getStackTraceString(tr)); }
}
