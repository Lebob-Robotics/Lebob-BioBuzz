package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Iterator;

/** Hardware the robot runs without: each lookup gives null when the robot configuration doesn't have the device. */
public final class OptionalHardware {
  private OptionalHardware() {}

  /** The device called name, or null. Through PsiKit's map it is the same logged device that get() returns. */
  public static <T> T find(HardwareMap hardwareMap, Class<? extends T> type, String name) {
    return hardwareMap.tryGet(type, name);
  }

  /** The first of these devices, or null when there are none. */
  public static <T> T first(Iterable<T> devices) {
    Iterator<T> it = devices.iterator();
    return it.hasNext() ? it.next() : null;
  }
}
