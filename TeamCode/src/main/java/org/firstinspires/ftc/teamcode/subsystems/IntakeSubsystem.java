package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.servo.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the intake motor and read the two beam breaks, which
 * both look at the last (4th) tunnel position and so tell when we are full.
 */
public class IntakeSubsystem extends SubsystemBase {
  /** False until the beam breaks are wired and in the config. Without them the intake never reports full. */
  private static final boolean BEAM_BREAKS_WIRED = false;
  /**
   * The intake has run in power mode since 27 Oct: under speed control it surged on and off, because
   * its encoder was unplugged or counted the wrong way. Set true once the log shows its velocity
   * rising when it intakes.
   */
  private static final boolean INTAKE_USES_ENCODER = false;
  /** Loops a beam reading must hold before "full" changes, to ignore flicker. */
  private static final int FULL_DEBOUNCE_LOOPS = 3; // TODO tune on robot

  private final GobildaMotor intakeMotor;
  private final DigitalChannel beamBreakTop;
  private final DigitalChannel beamBreakBottom;

  public enum IntakeState {
    STOP, INTAKE, EJECT
  }

  private IntakeState intakeState;
  private IntakeState previousIntakeState = IntakeState.STOP;

  private boolean full = false;
  private int fullChangeLoops = 0;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    intakeMotor = INTAKE_USES_ENCODER
        ? GobildaMotor.speedControlled(hardwareMap, "IntakeMotor", false, false)
        : GobildaMotor.powerControlled(hardwareMap, "IntakeMotor", false, false);
    beamBreakTop = BEAM_BREAKS_WIRED ? createBeamBreak(hardwareMap, "BeamBreakT") : null;
    beamBreakBottom = BEAM_BREAKS_WIRED ? createBeamBreak(hardwareMap, "BeamBreakB") : null;
    setIntakeState(IntakeState.STOP);
  }

  private static DigitalChannel createBeamBreak(HardwareMap hardwareMap, String name) {
    DigitalChannel beamBreak = hardwareMap.get(DigitalChannel.class, name);
    beamBreak.setMode(DigitalChannel.Mode.INPUT);
    return beamBreak;
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
    return BEAM_BREAKS_WIRED && !beamBreakTop.getState();
  }

  /** True when a ball blocks the bottom beam. The receiver output goes low when broken. */
  public boolean isBottomBroken() {
    return BEAM_BREAKS_WIRED && !beamBreakBottom.getState();
  }

  /**
   * True when a ball sits in the last tunnel position, so the tunnel holds 4.
   * Either beam counts, so one failed sensor errs toward stopping the intake.
   */
  public boolean isFull() {
    return full;
  }

  /** Debounces the beams into {@link #full}. Counted in loops so replay matches. */
  private void updateFull() {
    boolean raw = isTopBroken() || isBottomBroken();
    if (raw == full) {
      fullChangeLoops = 0;
      return;
    }
    fullChangeLoops++;
    if (fullChangeLoops >= FULL_DEBOUNCE_LOOPS) {
      full = raw;
      fullChangeLoops = 0;
    }
  }

  @Override
  public void periodic() {
    updateFull();
    Logger.recordOutput("Intake/State", intakeState);
    Logger.recordOutput("Intake/BeamBreakTop", isTopBroken());
    Logger.recordOutput("Intake/BeamBreakBottom", isBottomBroken());
    Logger.recordOutput("Intake/Full", full);
    Logger.recordOutput("Intake/Velocity", intakeMotor.getVelocity());
    Logger.recordOutput("Intake/CommandedPower", intakeMotor.getCommandedPower());
  }
}
