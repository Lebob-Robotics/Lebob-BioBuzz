package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the feeder tunnel, which holds all the balls. It is driven by one
 * Melonbotics Super Servo in continuous mode.
 */
public class FeederSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private static final double EJECT_POWER = 1.0;

  // ---- Hardware --------------------------------------------------------
  private final CrServo feederServo;

  public enum FeederState {
    STOP, FEED, EJECT
  }

  private FeederState feederState;
  private FeederState previousFeederState = FeederState.STOP;

  public FeederSubsystem(HardwareMap hardwareMap) {
    // Reversed: on Mark 3 a positive raw power runs the tunnel backward
    // (Servo Check and driving, 27 Oct).
    feederServo = new CrServo(hardwareMap, "FeederServo", true);

    setFeederState(FeederState.STOP);
  }

  private void doFeederState() {
    if (previousFeederState == feederState) {
      return;
    }

    switch (feederState) {
      case STOP:
        feederServo.stop();
        break;
      case FEED:
        feederServo.setPower(FEED_POWER);
        break;
      case EJECT:
        feederServo.setPower(-EJECT_POWER);
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
    Logger.recordOutput("Feeder/CommandedPower", feederServo.getCommandedPower());
  }
}
