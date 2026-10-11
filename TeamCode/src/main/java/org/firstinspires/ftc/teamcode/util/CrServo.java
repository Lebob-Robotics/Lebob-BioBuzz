package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CrServo {
  /** Power changes smaller than this are not re-sent to the hub. */
  private static final double POWER_MIN = 1e-3;

  private final String name;
  private final CRServo servo;
  private double lastPower = Double.NaN;

  /**
   * @param hardwareMap the OpMode's hardware map
   * @param name        the device name in the robot configuration
   * @param reversed    true to flip the servo's positive direction
   */
  public CrServo(HardwareMap hardwareMap, String name, boolean reversed) {
    this.name = name;
    this.servo = hardwareMap.get(CRServo.class, name);
    servo.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  /**
   * Sets servo power, clamped to [-1, 1]. Skips the hardware write if unchanged.
   */
  public void setPower(double power) {
    power = Math.max(-1.0, Math.min(1.0, power));
    if (!Double.isNaN(lastPower) && Math.abs(power - lastPower) < POWER_MIN) {
      return;
    }
    lastPower = power;
    servo.setPower(power);
  }

  public void stop() {
    setPower(0.0);
  }

  public double getPower() {
    return servo.getPower();
  }

  public String getName() {
    return name;
  }
}
