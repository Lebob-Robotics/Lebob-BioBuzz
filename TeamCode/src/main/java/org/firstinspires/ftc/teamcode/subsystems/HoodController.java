package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Hood position control in lift degrees, stepped by {@link HoodLoop} at a few hundred hertz on the
 * REV Through Bore (the hood itself, past the belt's slack).
 *
 * <p>A trapezoidal profile is re-planned from its own state every step, so a target that moves each
 * loop (shooting on the move) is followed without restarting. Feedforward drives the profile; a PD on
 * the REV corrects what it misses. Once the profile has arrived and the hood is inside the deadband,
 * power is cut and the motor's brake holds it: a loop that keeps pushing across the belt's slack hunts.
 *
 * <p>Each move starts by taking up the belt's slack: the motor turns about 1.5 degrees before the hood
 * moves. A profile started at once ran ahead during that, and the hood, which accelerates at about
 * 10000 deg/s^2, then caught up past it and overshot by a degree. So the profile waits until the hood
 * moves, then starts from where it is. A hood that friction stops short of the goal starts again the
 * same way: pushing harder until it broke free jumped it past the goal.
 *
 * <p>The hub's own RUN_TO_POSITION could not do this: its speed loop either swung 3-6 degrees around
 * the target or crept and stopped short (Hood Tune sweeps, 5 Nov).
 */
public final class HoodController {
  /** Tuning, in lift degrees, seconds and motor power (-1..1 at 12 V). */
  public static final class Gains {
    public double maxVelDegPerSec;
    public double accelDegPerSec2;
    public double decelDegPerSec2;
    /** Power that just overcomes friction, applied in the direction of travel. */
    public double kS;
    /** Power per degree per second. */
    public double kV;
    /** Power per degree per second squared while speeding up. */
    public double kA;
    /**
     * The same while braking. Higher: too little braking arrives past the goal, while too much starting
     * kick on a short move overshoots it too, so each side gets its own.
     */
    public double kADecel;
    /** Power that holds the hood against gravity (positive raises it). */
    public double kG;
    /** Power per degree of position error. */
    public double kP;
    /** Power per degree per second of speed error. */
    public double kD;
    /** Settled inside this; re-engages once the error grows past {@link #reengageDeg}. */
    public double deadbandDeg;
    public double reengageDeg;
    /** The profile restarts from the hood when it falls this far behind (a stall or a hand on it). */
    public double resyncDeg;
    public double maxPower;
    /**
     * Power that winds the motor through the belt's slack at the start of a move: kS plus this much per
     * degree to go, rising by takeUpRampPerSec until the hood moves, up to the cap. Full power on a
     * half-degree move had the hood at 70 deg/s, too fast to stop in time, by the time it read as moving.
     */
    public double takeUpPerDeg;
    public double takeUpRampPerSec;
    public double takeUpPower;
    /** The hood counts as moving once this fast toward the goal, or this far along. */
    public double takeUpVelDegPerSec;
    public double takeUpDeg;
    /** Starts the profile anyway after this long. */
    public double takeUpMaxS;
    /** Stopped this long outside the deadband after the profile has arrived: the move starts again. */
    public double stuckS;
    /**
     * Read-to-drive delay: the hood is judged where it will be by the time the power acts. At 200
     * deg/s, 5 ms is a degree, which uncorrected reads as lagging the profile and drives it past.
     */
    public double latencyS;
  }

  private double setpointDeg;
  private double setpointVel;
  private double setpointAccel;
  private boolean settled = true;
  private boolean started;
  private boolean takingUp;
  private double takeUpDir;
  private double takeUpStartDeg;
  private double takeUpS;
  private double stuckForS;
  private static final double STUCK_DEG_PER_SEC = 5;

  /** Forgets the profile: the next step starts from wherever the hood is. */
  public void reset() {
    started = false;
    settled = true;
    takingUp = false;
    setpointVel = 0;
    setpointAccel = 0;
  }

  /**
   * One control step.
   *
   * @param dtS time since the last step
   * @param measuredDeg hood lift as last read
   * @param velDegPerSec hood speed now
   * @param goalDeg where the hood should be
   * @return motor power, positive raising the hood, before battery compensation
   */
  public double update(double dtS, double measuredDeg, double velDegPerSec, double goalDeg, Gains g) {
    double posDeg = measuredDeg + velDegPerSec * g.latencyS;
    if (!started) {
      started = true;
      setpointDeg = posDeg;
      setpointVel = 0;
      settled = true;
    }
    if (settled) {
      if (Math.abs(goalDeg - posDeg) <= g.reengageDeg) {
        setpointDeg = posDeg;
        setpointVel = 0;
        setpointAccel = 0;
        return 0;
      }
      settled = false;
      startTakeUp(posDeg, goalDeg);
    }
    if (takingUp) {
      takeUpS += dtS;
      boolean moving = velDegPerSec * takeUpDir >= g.takeUpVelDegPerSec
          || (posDeg - takeUpStartDeg) * takeUpDir >= g.takeUpDeg;
      if (!moving && takeUpS < g.takeUpMaxS) {
        return takeUpPower(posDeg, goalDeg, g);
      }
      takingUp = false;
      setpointDeg = posDeg;
      setpointVel = takeUpDir * Math.min(g.maxVelDegPerSec, Math.max(0, velDegPerSec * takeUpDir));
    }
    if (Math.abs(setpointDeg - posDeg) > g.resyncDeg) {
      setpointDeg = posDeg;
      setpointVel = velDegPerSec;
    }
    stepProfile(dtS, goalDeg, g);

    double error = setpointDeg - posDeg;
    boolean profileDone = setpointDeg == goalDeg && setpointVel == 0;
    if (profileDone && Math.abs(goalDeg - posDeg) <= g.deadbandDeg && Math.abs(velDegPerSec) < g.maxVelDegPerSec * 0.05) {
      settled = true;
      return 0;
    }
    stuckForS = profileDone && Math.abs(velDegPerSec) < STUCK_DEG_PER_SEC ? stuckForS + dtS : 0;
    if (stuckForS >= g.stuckS) {
      stuckForS = 0;
      // Breaking free near the top jumped the hood 0.2-0.3 degrees, past a tighter band and back
      if (Math.abs(goalDeg - posDeg) <= g.reengageDeg) {
        settled = true;
        return 0;
      }
      startTakeUp(posDeg, goalDeg);
      takeUpS = dtS;
      return takeUpPower(posDeg, goalDeg, g);
    }
    boolean braking = setpointAccel * setpointVel < 0;
    double power = g.kV * setpointVel + (braking ? g.kADecel : g.kA) * setpointAccel + g.kG
        + g.kP * error + g.kD * (setpointVel - velDegPerSec);
    // Friction is beaten in the direction the profile moves, or, once it has arrived, toward the goal
    double frictionSign = setpointVel != 0 ? Math.signum(setpointVel) : Math.signum(error);
    power += g.kS * frictionSign;
    return Math.max(-g.maxPower, Math.min(g.maxPower, power));
  }

  private void startTakeUp(double posDeg, double goalDeg) {
    takingUp = true;
    takeUpDir = Math.signum(goalDeg - posDeg);
    takeUpStartDeg = posDeg;
    takeUpS = 0;
    stuckForS = 0;
  }

  private double takeUpPower(double posDeg, double goalDeg, Gains g) {
    setpointDeg = posDeg;
    setpointVel = 0;
    setpointAccel = 0;
    double p = g.kS + g.takeUpPerDeg * Math.abs(goalDeg - posDeg) + g.takeUpRampPerSec * takeUpS;
    return takeUpDir * Math.min(g.takeUpPower, p);
  }

  /**
   * Moves the setpoint one step toward the goal within the speed and acceleration limits. Braking uses
   * exactly the deceleration that stops on the goal: a fixed rate entered a step late (up to a degree
   * at full speed) never catches up and arrives still moving.
   */
  private void stepProfile(double dtS, double goalDeg, Gains g) {
    double dist = goalDeg - setpointDeg;
    double dir = Math.signum(dist);
    double v = setpointVel;
    if (dir == 0) {
      land(goalDeg);
      return;
    }
    double newVel;
    if (v * dir < 0) { // Still moving the other way (the goal jumped back): brake first
      newVel = v + dir * g.decelDegPerSec2 * dtS;
      if (newVel * dir > 0) {
        newVel = 0;
      }
    } else {
      double speed = Math.abs(v);
      double stopDecel = speed * speed / (2 * Math.abs(dist));
      // Never faster than still stops on the goal at the set deceleration after this step's travel;
      // without this a short move kept accelerating and then had to brake at three times that rate
      double halfStep = 0.5 * g.decelDegPerSec2 * dtS;
      double stoppable = -halfStep + Math.sqrt(halfStep * halfStep + 2 * g.decelDegPerSec2 * Math.abs(dist));
      speed = stopDecel >= g.decelDegPerSec2
          ? Math.max(0, speed - stopDecel * dtS)
          : Math.min(Math.min(g.maxVelDegPerSec, stoppable), speed + g.accelDegPerSec2 * dtS);
      newVel = dir * speed;
    }
    double newPos = setpointDeg + 0.5 * (v + newVel) * dtS;
    if ((goalDeg - newPos) * dir <= 0) {
      land(goalDeg);
      return;
    }
    setpointAccel = (newVel - v) / dtS;
    setpointDeg = newPos;
    setpointVel = newVel;
  }

  private void land(double goalDeg) {
    setpointDeg = goalDeg;
    setpointVel = 0;
    setpointAccel = 0;
  }

  public double setpointDeg() { return setpointDeg; }
  public double setpointVelDegPerSec() { return setpointVel; }
  public boolean isSettled() { return settled; }
  public boolean isTakingUp() { return takingUp; }
}
