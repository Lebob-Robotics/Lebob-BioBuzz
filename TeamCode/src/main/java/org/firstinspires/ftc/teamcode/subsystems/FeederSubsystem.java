package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CrServo;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.hardware.OptionalHardware;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the feeder after the independent last indexer roller. The current three-servo
 * robot uses FeederServo on Control Hub port 1; the two other servos belong to the indexer.
 * Optional L/R names are retained for configurations with a paired feeder drive.
 * {@link FeederState#HOLD} runs it backward so the indexer can queue balls
 * against it without any reaching the shooter.
 */
public class FeederSubsystem extends StatePowerSubsystem<FeederSubsystem.FeederState> {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
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

  public FeederSubsystem(HardwareMap hardwareMap) {
    super(FEED_POWER); // A volley's (Lebot Teleop RB) feed power can be lower
    // R is the original feeder servo. Reversed: on Mark 3 a positive raw power
    // runs the tunnel backward (Servo Check and driving, 27 Oct).
    // Current three-servo robot uses the legacy FeederServo name on Control Hub port 1.
    CrServo r = find(hardwareMap, HardwareNames.FEEDER_SERVO_RIGHT, true);
    feederServoR = r != null ? r : find(hardwareMap, HardwareNames.FEEDER_SERVO, true);
    // Optional opposite-facing paired output; absent on the current three-servo robot.
    feederServoL = find(hardwareMap, HardwareNames.FEEDER_SERVO_LEFT, false);

    setFeederState(FeederState.STOP);
  }

  /** The servo named name, or null if the robot configuration has none. */
  private static CrServo find(HardwareMap hardwareMap, String name, boolean reversed) {
    return OptionalHardware.find(hardwareMap, CRServo.class, name) == null
        ? null : new CrServo(hardwareMap, name, reversed);
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
    setFeederState(newFeederState, feedPower());
  }

  /** Set state and power together, so returning to HOLD never briefly writes forward power. */
  public void setFeederState(FeederState newFeederState, double power) {
    setState(newFeederState, power);
  }

  @Override
  void write(FeederState state, double feedPower) {
    setFeederPower(powerFor(state, feedPower));
  }

  public FeederState getFeederState() {
    return state();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Feeder/State", state());
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
