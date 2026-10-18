package org.firstinspires.ftc.teamcode;

public enum Alliance {
  RED, BLUE;

  /** Cluster names start with the alliance colour, for example "RED SCORING". */
  public boolean ownsCluster(String clusterName) {
    return clusterName.startsWith(name());
  }
}
