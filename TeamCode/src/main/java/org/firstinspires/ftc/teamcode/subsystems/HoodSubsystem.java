package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;

/**
 * Subsystem to control the shooter hood.
 *
 * <p>The hood is driven by a goBILDA 435 RPM Yellow Jacket. A REV Through Bore
 * encoder (8192 ticks/rev) sits on the hood pinion and is plugged into the
 * HoodMotor's encoder port, so {@link GobildaMotor#getPosition()} reads the
 * hood directly.
 *
 * <p>We run our own P loop in {@link #periodic()} instead of RUN_TO_POSITION.
 * The built-in position PID is tuned for the motor's own encoder, not an
 * 8192 CPR one, so it would behave badly with this feedback.
 */
public class HoodSubsystem extends SubsystemBase {
  /** Proportional gain, in motor power per encoder tick of error. */
  private static final double KP = 0.002; // TODO tune on robot
  /** Largest power the P loop may command. */
  private static final double MAX_POWER = 0.6;
  /** Errors smaller than this (ticks) are treated as on target. */
  private static final int DEADBAND_TICKS = 30;
  /** Soft limit at the lower hard stop, where the encoder is zeroed. */
  private static final int MIN_TICKS = 0;
  /** Soft limit at the top of the hood's travel. */
  private static final int MAX_TICKS = 2000; // TODO tune on robot

  private final GobildaMotor hoodMotor;

  /** Hood positions, each holding a target in Through Bore ticks. */
  public enum HoodPreset {
    STOW(0), // TODO tune on robot
    CLOSE(500), // TODO tune on robot
    MID(1000), // TODO tune on robot
    FAR(1500); // TODO tune on robot

    private final int ticks;

    HoodPreset(int ticks) {
      this.ticks = ticks;
    }

    public int getTicks() {
      return ticks;
    }
  }

  private HoodPreset preset = HoodPreset.STOW;

  public HoodSubsystem(HardwareMap hardwareMap) {
    // reversed=false is a guess: verify on robot that positive power raises the hood.
    hoodMotor = new GobildaMotor(hardwareMap, "HoodMotor", false, DcMotor.RunMode.RUN_WITHOUT_ENCODER, true);
  }

  /**
   * Called once from the OpMode's init(). Zeroes the encoder, so the hood must
   * be resting at its lower hard stop when this runs.
   */
  public void init() {
    hoodMotor.resetEncoder();
  }

  @Override
  public void periodic() {
    int position = hoodMotor.getPosition();
    int error = getTarget() - position;

    if (Math.abs(error) < DEADBAND_TICKS) {
      hoodMotor.stop();
      return;
    }

    double power = Math.max(-MAX_POWER, Math.min(MAX_POWER, KP * error));

    // Soft limits: never drive further down below MIN_TICKS or up above MAX_TICKS.
    if ((power < 0 && position <= MIN_TICKS) || (power > 0 && position >= MAX_TICKS)) {
      hoodMotor.stop();
      return;
    }

    hoodMotor.setPower(power);
  }

  public void setPreset(HoodPreset newPreset) {
    preset = newPreset;
  }

  public HoodPreset getPreset() {
    return preset;
  }

  /** Hood position in Through Bore ticks. */
  public int getPosition() {
    return hoodMotor.getPosition();
  }

  /** Target position in ticks for the current preset. */
  public int getTarget() {
    return preset.getTicks();
  }

  /** True when the hood is within the deadband of its target. */
  public boolean atTarget() {
    return Math.abs(getTarget() - getPosition()) < DEADBAND_TICKS;
  }
}
