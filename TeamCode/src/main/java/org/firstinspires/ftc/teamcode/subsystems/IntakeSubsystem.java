package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.utils.MotorReversalGuard;

import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the intake motor and read the two beam breaks, which
 * both look at the last (4th) tunnel position and so tell when we are full.
 */
public class IntakeSubsystem extends SubsystemBase {
  /**
   * The beam breaks are no longer used (13 Nov): never read, so they need not be in the configuration. The
   * robot tells it is full by a loading ball touching the flywheels instead ({@code WheelContact}).
   */
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
  private final ElapsedTime clock = new ElapsedTime();
  // Intake encoder is untrusted: allow 0.75 s to coast, then ramp for 0.75 s.
  private final MotorReversalGuard reversal = new MotorReversalGuard(0.75, 0.75, 0);

  private final IntakeOverloadGuard overload = new IntakeOverloadGuard();
  private double amps;

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
    if (overload.isTripped()) {
      reversal.update(0, clock.seconds(), 0);
      intakeMotor.cut();
      return;
    }
    int direction = intakeState == IntakeState.INTAKE ? 1 : intakeState == IntakeState.EJECT ? -1 : 0;
    double scale = reversal.update(direction, clock.seconds(), 0);
    intakeMotor.setPower(direction * scale);
  }

  public void setIntakeState(IntakeState newIntakeState) {
    intakeState = newIntakeState;
    doIntakeState();
  }

  /** Full capacity rejects incoming balls; otherwise collection is continuous. */
  public static IntakeState automaticState(boolean full, boolean ejecting) {
    return full || ejecting ? IntakeState.EJECT : IntakeState.INTAKE;
  }

  /** As above, but the intake only runs while the driver asks for it (LT); eject always runs. */
  public static IntakeState automaticState(boolean full, boolean ejecting, boolean wanted) {
    if (ejecting) {
      return IntakeState.EJECT;
    }
    return wanted ? automaticState(full, false) : IntakeState.STOP;
  }

  public IntakeState getIntakeState() {
    return overload.isTripped() ? IntakeState.STOP : intakeState;
  }

  public boolean isOverloaded() { return overload.isTripped(); }

  /** Last current read, A; NaN if the read failed. */
  public double getAmps() { return amps; }

  /** Share of the overload guard's load budget used, 0-1: it trips at 1. */
  public double getLoadFraction() { return overload.getLoad() / IntakeOverloadGuard.LOAD_BUDGET; }

  /** Left-bumper rising edge, after clearing the obstruction and allowing the motor to cool. */
  public void resetOverload() {
    overload.reset(clock.seconds());
  }

  public String getOverloadStatus() {
    if (overload.isLockedOut()) {
      return "OVERLOAD LOCKOUT: inspect intake; restart OpMode after cooling";
    }
    if (overload.isTripped()) {
      double remaining = overload.cooldownRemaining(clock.seconds());
      return String.format("OVERLOAD %d/%d (%s): %s", overload.getTrips(), IntakeOverloadGuard.MAX_TRIPS,
          overload.getReason(), remaining > 0 ? String.format("cooldown %.0f s", Math.ceil(remaining))
              : "clear jam, cool motor, tap LB to reset");
    }
    return String.format("ok (%.1f A)", amps);
  }

  private void checkOverload() {
    boolean powered = Math.abs(intakeMotor.getCommandedPower()) > 0;
    try {
      amps = powered ? intakeMotor.getCurrentAmps() : 0;
    } catch (RuntimeException e) {
      amps = Double.NaN; // Fail closed if the current read fails.
    }
    boolean wasTripped = overload.isTripped();
    if (overload.update(clock.seconds(), powered, amps)) {
      intakeMotor.cut();
      if (!wasTripped) {
        RobotLog.ee("BioBuzz", "intake overload %d/%d: %s, %.2f A", overload.getTrips(),
            IntakeOverloadGuard.MAX_TRIPS, overload.getReason(), amps);
      }
    }
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
   * Either beam counts, so a blocked sensor errs toward rejecting extra balls.
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
    checkOverload();
    doIntakeState();
    Logger.recordOutput("Intake/State", getIntakeState());
    Logger.recordOutput("Intake/RequestedState", intakeState);
    Logger.recordOutput("Intake/Overload/Tripped", overload.isTripped());
    Logger.recordOutput("Intake/Overload/Trips", overload.getTrips());
    Logger.recordOutput("Intake/Overload/LoadBudget", overload.getLoad());
    Logger.recordOutput("Intake/Overload/Reason", overload.getReason());
    Logger.recordOutput("Intake/Overload/CooldownSeconds", overload.cooldownRemaining(clock.seconds()));
    Logger.recordOutput("Intake/BeamBreakTop", isTopBroken());
    Logger.recordOutput("Intake/BeamBreakBottom", isBottomBroken());
    Logger.recordOutput("Intake/Full", full);
    Logger.recordOutput("Intake/Velocity", intakeMotor.getVelocity());
    Logger.recordOutput("Intake/CommandedPower", intakeMotor.getCommandedPower());
    Logger.recordOutput("Intake/Amps", amps);
  }
}
