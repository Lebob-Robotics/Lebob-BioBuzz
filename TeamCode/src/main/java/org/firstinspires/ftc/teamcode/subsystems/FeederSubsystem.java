package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
  private final CrServo feederServoL;
  private final CrServo feederServoR;

  public enum FeederState {
    STOP, FEED, HOLD, EJECT
  }

  private FeederState feederState;
  private FeederState previousFeederState = FeederState.STOP;

  public FeederSubsystem(HardwareMap hardwareMap) {
    // R is the original feeder servo. Reversed: on Mark 3 a positive raw power
    // runs the tunnel backward (Servo Check and driving, 27 Oct).
    feederServoR = new CrServo(hardwareMap, "FeederServoR", true);
    // L sits on the other end of the shaft, so it faces the other way and is
    // not reversed. Direction is to be verified on the robot with Servo Check:
    // if the two servos fight each other, flip this.
    feederServoL = new CrServo(hardwareMap, "FeederServoL", false);

    setFeederState(FeederState.STOP);
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
    Logger.recordOutput("Feeder/CommandedPower", feederServoL.getCommandedPower());
  }

  /** Both servos drive the same shaft at the same ratio, so they get the same power. */
  private void setFeederPower(double power) {
    feederServoL.setPower(power);
    feederServoR.setPower(power);
  }
}
