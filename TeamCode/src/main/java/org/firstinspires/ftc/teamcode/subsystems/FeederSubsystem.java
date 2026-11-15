package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the feeder after the independent last indexer roller. The current three-servo
 * robot uses FeederServo on Control Hub port 1; the two other servos belong to the indexer.
 * Optional L/R names are retained for configurations with a paired feeder drive.
 * {@link FeederState#HOLD} runs it backward so the indexer can queue balls
 * against it without any reaching the shooter.
 */
public class FeederSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private double feedPower = FEED_POWER; // A volley's (Lebot Teleop RB) can be lower
  private static final double EJECT_POWER = 1.0;
  private static final double HOLD_POWER = 1.0;

  // ---- Hardware --------------------------------------------------------
  // Either can be null until the robot configuration has both servos: see the constructor.
  private final CrServo feederServoL;
  private final CrServo feederServoR;
  private double commandedPower = 0.0;

  public enum FeederState {
    STOP, FEED, HOLD, EJECT
  }

  private FeederState feederState;

  public FeederSubsystem(HardwareMap hardwareMap) {
    // R is the original feeder servo. Reversed: on Mark 3 a positive raw power
    // runs the tunnel backward (Servo Check and driving, 27 Oct).
    // Current three-servo robot uses the legacy FeederServo name on Control Hub port 1.
    CrServo r = find(hardwareMap, "FeederServoR", true);
    feederServoR = r != null ? r : find(hardwareMap, "FeederServo", true);
    // Optional opposite-facing paired output; absent on the current three-servo robot.
    feederServoL = find(hardwareMap, "FeederServoL", false);

    setFeederState(FeederState.STOP);
  }

  /** The servo named name, or null if the robot configuration has none. */
  private static CrServo find(HardwareMap hardwareMap, String name, boolean reversed) {
    return hardwareMap.tryGet(CRServo.class, name) == null ? null : new CrServo(hardwareMap, name, reversed);
  }

  static double powerFor(FeederState state, double feed) {
    switch (state) {
      case STOP:
        return 0.0;
      case FEED:
        return Math.max(0, Math.min(1, feed));
      case HOLD:
        return -HOLD_POWER;
      case EJECT:
        return -EJECT_POWER;
      default:
        throw new IllegalArgumentException("Unknown feeder state " + state);
    }
  }

  public void setFeederState(FeederState newFeederState) {
    setFeederState(newFeederState, feedPower);
  }

  /** Set state and power together, so returning to HOLD never briefly writes forward power. */
  public void setFeederState(FeederState newFeederState, double power) {
    double p = Math.max(0, Math.min(1, power));
    if (feederState == newFeederState && feedPower == p) { return; }
    feederState = newFeederState;
    feedPower = p;
    setFeederPower(powerFor(feederState, feedPower));
  }

  /** Power while feeding, 0-1; the default is full. Applies at once if feeding now. */
  public void setFeedPower(double power) {
    setFeederState(feederState, power);
  }

  public FeederState getFeederState() {
    return feederState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Feeder/State", feederState);
    Logger.recordOutput("Feeder/CommandedPower", commandedPower);
  }

  /** Apply the stage command to each configured feeder output. */
  private void setFeederPower(double power) {
    commandedPower = power;
    if (feederServoL != null) {
      feederServoL.setPower(power);
    }
    if (feederServoR != null) {
      feederServoR.setPower(power);
    }
  }
}
