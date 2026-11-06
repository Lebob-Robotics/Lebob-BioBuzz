package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the feeder, the last stage before the shooter. It is driven by two
 * Melonbotics Super Servos in continuous mode, both belted to the same feeder
 * shaft at the same gear ratio, so they always get the same power.
 * {@link FeederState#HOLD} runs it backward so the indexer can queue balls
 * against it without any reaching the shooter.
 */
public class FeederSubsystem extends SubsystemBase {
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

  private FeederState feederState;
  private FeederState previousFeederState = FeederState.STOP;

  public FeederSubsystem(HardwareMap hardwareMap) {
    // R is the original feeder servo. Reversed: on Mark 3 a positive raw power
    // runs the tunnel backward (Servo Check and driving, 27 Oct).
    // TODO: the robot configuration still names it "FeederServo" and has no
    // FeederServoL, which crashed the OpMode. Fall back until the configuration
    // is updated, then use get() for both again.
    CrServo r = find(hardwareMap, "FeederServoR", true);
    feederServoR = r != null ? r : find(hardwareMap, "FeederServo", true);
    // L sits on the other end of the shaft, so it faces the other way and is
    // not reversed. Direction is to be verified on the robot with Servo Check:
    // if the two servos fight each other, flip this.
    feederServoL = find(hardwareMap, "FeederServoL", false);

    setFeederState(FeederState.STOP);
  }

  /** The servo named name, or null if the robot configuration has none. */
  private static CrServo find(HardwareMap hardwareMap, String name, boolean reversed) {
    return hardwareMap.tryGet(CRServo.class, name) == null ? null : new CrServo(hardwareMap, name, reversed);
  }

  private void doFeederState() {
    if (previousFeederState == feederState) {
      return;
    }

    switch (feederState) {
      case STOP:
        setFeederPower(0.0);
        break;
      case FEED:
        setFeederPower(FEED_POWER);
        break;
      case HOLD:
        setFeederPower(-HOLD_POWER);
        break;
      case EJECT:
        setFeederPower(-EJECT_POWER);
        break;
    }
  }

  public void setFeederState(FeederState newFeederState) {
    previousFeederState = feederState;
    feederState = newFeederState;
    doFeederState();
  }

  public FeederState getFeederState() {
    return feederState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Feeder/State", feederState);
    Logger.recordOutput("Feeder/CommandedPower", commandedPower);
  }

  /** Both servos drive the same shaft at the same ratio, so they get the same power. */
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
