package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Thin wrapper around a {@link DcMotorEx}, shared by every subsystem that
 * drives a motor (drivetrain, intake, shooter, indexer, ...).
 *
 * <p>
 * Configuration methods return {@code this} so a motor can be set up in one
 * expression:
 *
 * <pre>{@code
 * GobildaMotor shooter = new GobildaMotor(hardwareMap, "shooter")
 *     .setReversed(true)
 *     .setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
 *     .setBrakeOnZeroPower(false);
 * }</pre>
 */
public class GobildaMotor {
  /** Power changes smaller than this are not re-sent to the hub. */
  private static final double POWER_MIN = 1e-3;

  private final String name;
  private final DcMotorEx motor;
  private double lastPower = Double.NaN;

  /**
   * @param hardwareMap the OpMode's hardware map
   * @param name        the device name in the robot configuration
   */
  public GobildaMotor(HardwareMap hardwareMap, String name, boolean reversed, DcMotor.RunMode mode, boolean brake) {
    this.name = name;
    this.motor = hardwareMap.get(DcMotorEx.class, name);
    this.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    this.setMode(mode);
    this.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  /** Zeroes the encoder, then restores the previous run mode. */
  public GobildaMotor resetEncoder() {
    DcMotor.RunMode previous = motor.getMode();
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(previous);
    return this;
  }

  /*
   * Sets motor power, clamped to [-1, 1]. Skips the hardware write if unchanged.
   */
  public void setPower(double power) {
    power = Math.max(-1.0, Math.min(1.0, power));
    if (!Double.isNaN(lastPower) && Math.abs(power - lastPower) < POWER_EPSILON) {
      return;
    }
    lastPower = power;
    motor.setPower(power);
  }

  /** Sets a target velocity in ticks/second (requires RUN_USING_ENCODER). */
  public void setVelocity(double ticksPerSecond) {
    lastPower = Double.NaN; // power cache is no longer meaningful
    motor.setVelocity(ticksPerSecond);
  }

  public void stop() {
    setPower(0.0);
  }

  // ---------------------------------------------------------------- state

  public double getPower() {
    return motor.getPower();
  }

  /** Encoder position in ticks. */
  public int getPosition() {
    return motor.getCurrentPosition();
  }

  /** Velocity in ticks/second. */
  public double getVelocity() {
    return motor.getVelocity();
  }

  public String getName() {
    return name;
  }

  /** Escape hatch for anything not wrapped here. */
  public DcMotorEx getRaw() {
    return motor;
  }
}
