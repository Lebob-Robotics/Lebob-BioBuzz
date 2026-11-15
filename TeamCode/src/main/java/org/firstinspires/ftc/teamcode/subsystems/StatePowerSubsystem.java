package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

/**
 * A mechanism that is in one state at a time and runs at a feed power. State and power are set together, so
 * changing both never writes the power in between, and a set that changes neither writes nothing.
 */
abstract class StatePowerSubsystem<S extends Enum<S>> extends SubsystemBase {
  private S state;
  private double feedPower;

  StatePowerSubsystem(double feedPower) {
    this.feedPower = feedPower;
  }

  /** Power is clamped to 0-1. The outputs are written only when the state or the power changed. */
  final void setState(S newState, double power) {
    double p = Math.max(0, Math.min(1, power));
    if (state == newState && feedPower == p) { return; }
    state = newState;
    feedPower = p;
    write(state, feedPower);
  }

  final S state() {
    return state;
  }

  final double feedPower() {
    return feedPower;
  }

  /** Drives the outputs for this state at this feed power. */
  abstract void write(S state, double feedPower);
}
