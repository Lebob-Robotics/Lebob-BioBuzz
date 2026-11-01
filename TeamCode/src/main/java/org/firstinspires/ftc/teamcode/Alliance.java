package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;

public enum Alliance {
  RED, BLUE;

  /** Cluster names start with the alliance colour, for example "RED SCORING". */
  public boolean ownsCluster(String clusterName) {
    return clusterName.startsWith(name());
  }

  /** The field heading this alliance's driver calls "forward", standing at their wall. */
  public double forwardHeadingRad() {
    return this == RED
        ? ShootOnTheMoveConstants.ALLIANCE_FORWARD_RED_RAD
        : ShootOnTheMoveConstants.ALLIANCE_FORWARD_BLUE_RAD;
  }
}
