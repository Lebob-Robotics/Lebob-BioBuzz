package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;

/**
 * Subsystem to control the intake motor and read the two intake beam breaks.
 * The beam breaks are telemetry only for now.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final GobildaMotor intakeMotor;
  private final DigitalChannel beamBreakTop;
  private final DigitalChannel beamBreakBottom;

  public enum IntakeState {
    STOP, INTAKE, EJECT
  }

  private IntakeState intakeState;
  private IntakeState previousIntakeState = IntakeState.STOP;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    intakeMotor = new GobildaMotor(hardwareMap, "IntakeMotor", true, DcMotor.RunMode.RUN_USING_ENCODER, false);
    beamBreakTop = hardwareMap.get(DigitalChannel.class, "BeamBreakT");
    beamBreakBottom = hardwareMap.get(DigitalChannel.class, "BeamBreakB");
    beamBreakTop.setMode(DigitalChannel.Mode.INPUT);
    beamBreakBottom.setMode(DigitalChannel.Mode.INPUT);
    setIntakeState(IntakeState.STOP);
  }

  private void doIntakeState() {
    if (previousIntakeState == intakeState) {
      return;
    }

    switch (intakeState) {
      case STOP:
        intakeMotor.stop();
        break;
      case INTAKE:
        intakeMotor.setPower(1.0);
        break;
      case EJECT:
        intakeMotor.setPower(-1.0);
        break;
    }
  }

  public void setIntakeState(IntakeState newIntakeState) {
    previousIntakeState = intakeState;
    intakeState = newIntakeState;
    doIntakeState();
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }

  /** True when a ball blocks the top beam. The receiver output goes low when broken. */
  public boolean isTopBroken() {
    return !beamBreakTop.getState();
  }

  /** True when a ball blocks the bottom beam. The receiver output goes low when broken. */
  public boolean isBottomBroken() {
    return !beamBreakBottom.getState();
  }
}
