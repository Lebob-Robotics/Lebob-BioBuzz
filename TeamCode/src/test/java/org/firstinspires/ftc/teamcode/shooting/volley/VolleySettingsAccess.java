package org.firstinspires.ftc.teamcode.shooting.volley;

import java.util.List;

/** Test access to VolleySettings.fromLines from other packages. */
public final class VolleySettingsAccess {
  private VolleySettingsAccess() { }

  public static VolleySettings fromLines(List<String> lines) { return VolleySettings.fromLines(lines); }

  public static boolean isConfirmed(VolleySettings v, int spot) { return v.isConfirmed(spot); }

  public static double rate(VolleySettings v, int spot) { return v.rate(spot); }
}
