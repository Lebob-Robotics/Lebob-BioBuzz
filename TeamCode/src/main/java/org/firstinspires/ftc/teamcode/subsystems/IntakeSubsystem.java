package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the intake motor. The beam breaks are not wired, so
 * there is no ball sensing.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final GobildaMotor intakeMotor;

  public enum IntakeState {
    STOP, INTAKE, EJECT
  }

  private IntakeState intakeState;
  private IntakeState previousIntakeState = IntakeState.STOP;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    // Open loop: the intake needs no speed control, and RUN_USING_ENCODER surges
    // on and off when the encoder is unplugged or counts the wrong way.
    intakeMotor = new GobildaMotor(hardwareMap, "IntakeMotor", false, DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
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

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/State", intakeState);
    Logger.recordOutput("Intake/Velocity", intakeMotor.getVelocity());
    Logger.recordOutput("Intake/CommandedPower", intakeMotor.getCommandedPower());
  }
}
