package org.firstinspires.ftc.teamcode.field;

import java.util.List;

/**
 * Follows a {@link FieldRoute#route} leg by leg, boosting the stick the longer the robot makes no progress, and keeps
 * the clocks the drives stop on: time since the last tag fix and time on the route.
 */
public final class RouteDriver {
  private List<double[]> route;
  private int leg;
  private double legX;
  private double legY;
  private double startS;
  private double timeoutS;
  private int fixes;
  private double lastFixS;
  private double progressX;
  private double progressY;
  private double progressS;

  /** From the pose along the route; allowed GOTO_TIMEOUT_S plus 0.4 m/s over the route's length. */
  public void start(double nowS, FieldPose pose, List<double[]> route, int fixCount) {
    this.route = route;
    leg = 0;
    legX = pose.xM;
    legY = pose.yM;
    startS = nowS;
    double lengthM = 0;
    double fromX = pose.xM;
    double fromY = pose.yM;
    for (double[] w : route) {
      lengthM += Math.hypot(w[0] - fromX, w[1] - fromY);
      fromX = w[0];
      fromY = w[1];
    }
    timeoutS = FieldRoute.GOTO_TIMEOUT_S + lengthM / 0.4;
    fixes = fixCount;
    lastFixS = nowS;
    progressX = pose.xM;
    progressY = pose.yM;
    progressS = nowS;
  }

  /** Each loop: notes a new tag fix, and progress once the robot is 5 cm on from the last. */
  public void track(double nowS, FieldPose pose, int fixCount) {
    if (fixCount != fixes) {
      fixes = fixCount;
      lastFixS = nowS;
    }
    if (Math.hypot(pose.xM - progressX, pose.yM - progressY) >= 0.05) {
      progressX = pose.xM;
      progressY = pose.yM;
      progressS = nowS;
    }
  }

  /** Restarts the stuck boost and the tag-fix clock, for a scripted drive that is not the route. */
  public void restartClocks(double nowS) {
    progressS = nowS;
    lastFixS = nowS;
  }

  public boolean blind(double nowS) {
    return nowS - lastFixS > FieldRoute.GOTO_BLIND_S;
  }

  public boolean timedOut(double nowS) {
    return nowS - startS > timeoutS;
  }

  public double lastFixS() {
    return lastFixS;
  }

  /** Within GOTO_TOLERANCE_M of the route's end, on its last leg. */
  public boolean arrived(FieldPose pose) {
    double[] w = route.get(leg);
    return leg == route.size() - 1 && Math.hypot(w[0] - pose.xM, w[1] - pose.yM) <= FieldRoute.GOTO_TOLERANCE_M;
  }

  /** The stick toward this leg's end, moving on to the next leg within ROUTE_WAYPOINT_TOLERANCE_M of it. */
  public double[] stick(double nowS, FieldPose pose) {
    double[] w = route.get(leg);
    boolean last = leg == route.size() - 1;
    if (!last && Math.hypot(w[0] - pose.xM, w[1] - pose.yM) <= FieldRoute.ROUTE_WAYPOINT_TOLERANCE_M) {
      leg++;
      legX = pose.xM;
      legY = pose.yM;
      w = route.get(leg);
    }
    return FieldRoute.goToStick(pose, w[0], w[1], legX, legY, FieldRoute.GOTO_STUCK_BOOST_PER_S * (nowS - progressS));
  }
}
