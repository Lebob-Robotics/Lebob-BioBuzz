package org.firstinspires.ftc.teamcode.subsystems;

/** One short pulse each way, bounded independently by time, both encoders, and current. */
public final class HoodDirectionPulse {
  private int lastPhase;
  private int revStart;
  private int motorStart;
  private boolean cut;

  public double update(double seconds, int revTicks, int motorTicks, double amps) {
    int phase = seconds >= 0 && seconds < 0.15 ? 1
        : seconds >= 0.60 && seconds < 0.75 ? 2 : 0;
    if (phase != lastPhase) {
      revStart = revTicks;
      motorStart = motorTicks;
      cut = false;
      lastPhase = phase;
    }
    if (phase == 0) { return 0; }
    if (Math.abs(revTicks - revStart) >= 256 || Math.abs(motorTicks - motorStart) >= 20
        || !Double.isFinite(amps) || amps >= 3.0) {
      cut = true;
    }
    return cut ? 0 : phase == 1 ? 0.25 : -0.25;
  }
}
