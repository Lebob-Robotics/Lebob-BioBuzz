package org.firstinspires.ftc.teamcode.robot;

import java.util.List;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.vision.TagObservation;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;

/** The alliance the driver picks in INIT, and a warning while the cameras see only the other alliance's Hive. */
final class AlliancePicker {
  private final Robot robot;
  private final double[] hiveTagSeenS = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY}; // By Alliance ordinal

  AlliancePicker(Robot robot) {
    this.robot = robot;
  }

  void pollAlliance() {
    if (robot.driver.x) {
      robot.alliance = Alliance.BLUE;
    } else if (robot.driver.b) {
      robot.alliance = Alliance.RED;
    }
  }

  void noteHiveTags(List<TagObservation> observations) {
    for (TagObservation o : observations) {
      Alliance a = VisionCorrector.tagAlliance(o.id);
      if (a != null) {
        hiveTagSeenS[a.ordinal()] = robot.clock.seconds();
      }
    }
  }

  /** A warning while the cameras see only the other alliance's Hive, else null. */
  String allianceWarning() {
    Alliance other = robot.alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    double nowS = robot.clock.seconds();
    return allianceWarning(robot.alliance, nowS - hiveTagSeenS[robot.alliance.ordinal()],
        nowS - hiveTagSeenS[other.ordinal()]);
  }

  /**
   * Shoot Auto set to BLUE saw only RED's Hive tags and never fired: it ignores the other alliance's Hive (13 Nov,
   * 193125). Warns while the other Hive was seen in the last second and ours not in the last three.
   */
  static String allianceWarning(Alliance alliance, double ownSeenAgoS, double otherSeenAgoS) {
    Alliance other = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    if (otherSeenAgoS <= 1.0 && ownSeenAgoS > 3.0) {
      return "!! CAMERAS SEE " + other + "'S HIVE, NOT " + alliance + "'S. If you are " + other + ", press "
          + (other == Alliance.RED ? "B" : "X");
    }
    return null;
  }
}
