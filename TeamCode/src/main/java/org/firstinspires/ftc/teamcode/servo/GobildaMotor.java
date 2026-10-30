package org.firstinspires.ftc.teamcode.servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A motor that never runs faster than MAX_OUTPUT of its full speed. Motors use the hub's speed
 * control (RUN_USING_ENCODER), so setPower(p) asks for p of full speed, not p of the battery voltage.
 */
public class GobildaMotor {
  public static final double MAX_OUTPUT = 0.8; // Share of full speed any motor may be asked for
  private static final double POWER_MIN = 1e-3; // Power changes smaller than this are not re-sent to the hub

  private final String name;
  private final DcMotorEx motor;
  private final double maxVelocityTicksPerSec;
  private double lastPower = Double.NaN;
  private double commandedPower = 0.0;

  /**
   * A motor under the hub's speed control. Needs the motor's own encoder plugged in: without it the
   * hub sees no movement and pushes full power.
   *
   * @param name     the device name in the robot configuration
   * @param reversed true to flip the motor's positive direction
   * @param brake    true to brake at zero power, false to float
   */
  public static GobildaMotor speedControlled(HardwareMap hardwareMap, String name, boolean reversed, boolean brake) {
    return new GobildaMotor(hardwareMap, name, reversed, brake, DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /**
   * A motor driven by power alone. Only for a motor whose encoder port carries a different encoder
   * (the hood's Through Bore), where the hub's speed control would read the wrong speed. Power is
   * still capped at MAX_OUTPUT.
   */
  public static GobildaMotor powerControlled(HardwareMap hardwareMap, String name, boolean reversed, boolean brake) {
    return new GobildaMotor(hardwareMap, name, reversed, brake, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  private GobildaMotor(HardwareMap hardwareMap, String name, boolean reversed, boolean brake, DcMotor.RunMode mode) {
    this.name = name;
    this.motor = hardwareMap.get(DcMotorEx.class, name);
    motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    motor.setMode(mode);
    motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    maxVelocityTicksPerSec = MAX_OUTPUT * motor.getMotorType().getAchieveableMaxTicksPerSecond(); // From the motor type in the robot configuration
  }

  /** Zeroes the encoder, then restores the previous run mode. */
  public GobildaMotor resetEncoder() {
    DcMotor.RunMode previous = motor.getMode();
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(previous);
    return this;
  }

  /** Capped to MAX_OUTPUT either way. Skips the hardware write if unchanged. */
  public void setPower(double power) {
    power = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, power));
    commandedPower = power;
    if (!Double.isNaN(lastPower) && Math.abs(power - lastPower) < POWER_MIN) {
      return;
    }
    lastPower = power;
    motor.setPower(power);
  }

  /** Capped to {@link #getMaxVelocity()} either way. Speed-controlled motors only. */
  public void setVelocity(double ticksPerSecond) {
    lastPower = Double.NaN; // The power cache is no longer meaningful
    motor.setVelocity(Math.max(-maxVelocityTicksPerSec, Math.min(maxVelocityTicksPerSec, ticksPerSecond)));
  }

  /** MAX_OUTPUT of the motor's full speed, in ticks per second. */
  public double getMaxVelocity() {
    return maxVelocityTicksPerSec;
  }

  /** Velocity PIDF used in RUN_USING_ENCODER. */
  public PIDFCoefficients getVelocityPidf() {
    return motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /** Sets the velocity PIDF used in RUN_USING_ENCODER. */
  public void setVelocityPidf(double p, double i, double d, double f) {
    motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
  }

  public void stop() {
    setPower(0.0);
  }

  public double getPower() {
    return motor.getPower();
  }

  /** Last power requested via setPower. Kept in software so it logs and replays identically. */
  public double getCommandedPower() {
    return commandedPower;
  }

  /** Encoder position in ticks. */
  public int getPosition() {
    return motor.getCurrentPosition();
  }

  /** Velocity in ticks/second. */
  public double getVelocity() {
    return motor.getVelocity();
  }

  public double getCurrentAmps() {
    return motor.getCurrent(CurrentUnit.AMPS);
  }

  public String getName() {
    return name;
  }

  public DcMotorEx getRaw() {
    return motor;
  }
}
