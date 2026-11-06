package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Field;

/**
 * PsiKit's MotorWrapper passes power, mode, targets and encoder reads to the hub, but its inner
 * controller silently drops PIDF writes and target-position tolerance, and reads PIDF back as zeros
 * (checked in psikit-ftc 0.2.0). Gains must go to the hub's own motor behind the wrapper.
 */
public final class HubMotors {
  private HubMotors() {}

  /** The hub motor a wrapper holds, or the motor itself when it is not wrapped (or in replay). */
  public static DcMotorEx unwrap(DcMotorEx motor) {
    for (Field field : motor.getClass().getDeclaredFields()) {
      if (!DcMotorEx.class.isAssignableFrom(field.getType())) {
        continue;
      }
      try {
        field.setAccessible(true);
        Object inner = field.get(motor);
        if (inner instanceof DcMotorEx && inner != motor) {
          return (DcMotorEx) inner;
        }
      } catch (ReflectiveOperationException | RuntimeException ignored) {
        // Fall through to the wrapper; gains then keep their hub defaults.
      }
    }
    return motor;
  }
}
