package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class GobildaMotor {
  /** Power changes smaller than this are not re-sent to the hub. */
  private static final double POWER_MIN = 1e-3;

  private final String name;
  private final DcMotorEx motor;
  private double lastPower = Double.NaN;

  /**
   * @param hardwareMap the OpMode's hardware map
   * @param name        the device name in the robot configuration
   * @param reversed    true to flip the motor's positive direction
   * @param mode        run mode to configure the motor with
   * @param brake       true to brake at zero power, false to float
   */
  public GobildaMotor(HardwareMap hardwareMap, String name, boolean reversed, DcMotor.RunMode mode, boolean brake) {
    this.name = name;
    this.motor = hardwareMap.get(DcMotorEx.class, name);
    motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    motor.setMode(mode);
    motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  /** Zeroes the encoder, then restores the previous run mode. */
  public GobildaMotor resetEncoder() {
    DcMotor.RunMode previous = motor.getMode();
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(previous);
    return this;
  }

  /**
   * Sets motor power, clamped to [-1, 1]. Skips the hardware write if unchanged.
   */
  public void setPower(double power) {
    power = Math.max(-1.0, Math.min(1.0, power));
    if (!Double.isNaN(lastPower) && Math.abs(power - lastPower) < POWER_MIN) {
      return;
    }
    lastPower = power;
    motor.setPower(power);
  }

  /**
   * Changes the run mode if it differs, and forgets the cached power so the
   * next setPower reaches the motor in the new mode.
   */
  public void setMode(DcMotor.RunMode mode) {
    if (motor.getMode() == mode) {
      return;
    }
    motor.setMode(mode);
    lastPower = Double.NaN;
  }

  /** Sets a target velocity in ticks/second (requires RUN_USING_ENCODER). */
  public void setVelocity(double ticksPerSecond) {
    lastPower = Double.NaN; // power cache is no longer meaningful
    motor.setVelocity(ticksPerSecond);
  }

  public void stop() {
    setPower(0.0);
  }

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
