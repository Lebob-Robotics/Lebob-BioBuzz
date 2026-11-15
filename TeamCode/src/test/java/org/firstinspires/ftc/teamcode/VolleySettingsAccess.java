package org.firstinspires.ftc.teamcode;

import java.util.List;

/** Test access to VolleySettings.fromLines from other packages. */
public final class VolleySettingsAccess {
  private VolleySettingsAccess() { }

  public static VolleySettings fromLines(List<String> lines) { return VolleySettings.fromLines(lines); }
}
