package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CrServo;

/**
 * Controls the feeder tunnel, which holds all the balls. It is driven by two
 * Melonbotics Super Servos in continuous mode.
 */
public class FeederSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private static final double EJECT_POWER = 1.0;

  // ---- Hardware --------------------------------------------------------
  private final CrServo feederServoL;
  private final CrServo feederServoR;

  public enum FeederState {
    STOP, FEED, EJECT
  }

  private FeederState feederState;
  private FeederState previousFeederState = FeederState.STOP;

  public FeederSubsystem(HardwareMap hardwareMap) {
    // The servos face each other, so one is reversed. Direction is to be
    // verified on the robot.
    feederServoL = new CrServo(hardwareMap, "FeederServoL", true);
    feederServoR = new CrServo(hardwareMap, "FeederServoR", false);

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

  private void setFeederPower(double power) {
    feederServoL.setPower(power);
    feederServoR.setPower(power);
  }
}
