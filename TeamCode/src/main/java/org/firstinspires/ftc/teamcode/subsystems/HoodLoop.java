package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import java.lang.reflect.Method;

/**
 * Runs {@link HoodController} on its own thread, every few milliseconds instead of once per TeleOp loop
 * (about 80 ms, too slow for a hood that moves 200 degrees a second).
 *
 * <p>Each step sends its own bulk-read command to the hub with the REV Through Bore, rather than
 * {@code LynxModule.getBulkData()}, which would replace the main loop's per-loop cache mid-loop. Power
 * goes to the hub's motor behind PsiKit's wrapper, under a lock that {@link #deactivate()} shares, so
 * the main thread can always cut it.
 *
 * <p>The thread cuts power and ends when the main loop stops sending heartbeats (the OpMode stopped or
 * hung), and on repeated read errors, after which {@link #hasFailed()} sends the hood back to the hub's
 * own position loop.
 */
final class HoodLoop implements Runnable {
  private static final long HEARTBEAT_TIMEOUT_NS = 500_000_000L;
  private static final int MAX_CONSECUTIVE_ERRORS = 5;
  private static final long IDLE_PERIOD_MS = 20;
  private static final double VELOCITY_FILTER = 0.5; // Share of each new speed sample
  private static final int SAMPLE_CAPACITY = 256;
  // Pushing hard one way while the REV reads fast motion the other way: a reading with the wrong sign
  // would otherwise drive the hood into a stop. Braking never needs this much reverse power.
  private static final double WRONG_WAY_POWER = 0.5;
  private static final double WRONG_WAY_DEG_PER_SEC = 30;
  private static final double WRONG_WAY_SECONDS = 0.15;
  // Pushing hard while the REV reads no motion: a frozen REV reading (seen on the robot) or a jam. The
  // hood is otherwise moving well within this; left alone the loop drove it into the lower stop.
  private static final double STALL_POWER = 0.5;
  private static final double STALL_DEG_PER_SEC = 5;
  private static final double STALL_SECONDS = 0.12;
  /** A fresh REV read at activation may differ from the main loop's at rest by this many ticks. */
  private static final int ACTIVATION_TOLERANCE_TICKS = 100;
  /** Pushing up against the motor ceiling this long, short of the goal, means the encoders disagree. */
  private static final double CEILING_HOLD_SECONDS = 0.3;

  private final LynxModuleIntf revModule;
  private final int revPort;
  private final int revSign;
  private final DcMotorEx motor;
  private final double ticksPerDeg;
  private final Object powerLock = new Object();
  private final HoodController controller = new HoodController();
  private final HoodController.Gains gains = new HoodController.Gains();
  private final HoodFollowCheck follow = new HoodFollowCheck();
  private Thread thread;

  // Main thread to loop
  private volatile boolean active;
  private volatile boolean resetRequested;
  private volatile double openLoopPower = Double.NaN; // Set: drive this power instead of the controller
  private volatile double openLoopMinDeg;
  private volatile double openLoopMaxDeg;
  private volatile double goalDeg;
  private volatile int revZero;
  private volatile double minLiftDeg;
  private volatile double maxLiftDeg;
  private volatile double batteryVolts = 12.0;
  private volatile double speedScale = 1.0;
  private volatile long heartbeatNs;
  private volatile int motorHomeTicks;
  private volatile int motorCeilingTicks = Integer.MAX_VALUE;
  // Guarded by powerLock
  private double lastWritten = Double.NaN;

  // Loop to main thread
  private volatile double posDeg = Double.NaN;
  private volatile double velDegPerSec;
  private volatile double power;
  private volatile double setpointDeg = Double.NaN;
  private volatile double setpointVel;
  private volatile boolean settled;
  private volatile double periodMsAvg;
  private volatile double periodMsMax;
  private volatile String failure = "";
  private volatile boolean recoverable;
  private volatile int motorTicks;
  private volatile boolean atCeiling;
  private final Object sampleLock = new Object();
  private final double[][] samples = new double[5][SAMPLE_CAPACITY]; // t, pos, setpoint, vel, power
  private int sampleCount;

  private HoodLoop(LynxModuleIntf revModule, int revPort, int revSign, DcMotorEx motor, double ticksPerDeg) {
    this.revModule = revModule;
    this.revPort = revPort;
    this.revSign = revSign;
    this.motor = motor;
    this.ticksPerDeg = ticksPerDeg;
  }

  /**
   * A loop for the hood motor and the REV on the given encoder port, or null when they are not on a
   * Lynx hub (replay, or an SDK whose internals changed): the hood then keeps the hub's position loop.
   */
  static HoodLoop create(DcMotorEx rawHoodMotor, DcMotorEx rawRevEncoder, double ticksPerDeg) {
    try {
      if (!(rawRevEncoder.getController() instanceof LynxController) || !(rawRevEncoder instanceof DcMotorImpl)) {
        return null;
      }
      Method getModule = LynxController.class.getDeclaredMethod("getModule");
      getModule.setAccessible(true);
      LynxModuleIntf module = (LynxModuleIntf) getModule.invoke(rawRevEncoder.getController());
      // The bulk read is the raw count; the SDK flips it for a reversed motor, so do the same
      Method adjust = DcMotorImpl.class.getDeclaredMethod("adjustPosition", int.class);
      adjust.setAccessible(true);
      int sign = (Integer) adjust.invoke(rawRevEncoder, 1);
      return new HoodLoop(module, rawRevEncoder.getPortNumber(), sign, rawHoodMotor, ticksPerDeg);
    } catch (ReflectiveOperationException | RuntimeException e) {
      return null;
    }
  }

  // ---- Main thread ---------------------------------------------------------

  void heartbeat() { heartbeatNs = System.nanoTime(); }
  void setGoalDeg(double deg) { goalDeg = deg; }
  void setRevZero(int ticks) { revZero = ticks; }
  /**
   * The motor encoder's raw count on the lower stop, and how far above it the motor may drive the hood
   * up. Counted on the motor itself, never re-zeroed to the REV, so a REV that stops counting cannot
   * send the hood past its top.
   */
  void setMotorHome(int rawTicks, int ceilingTicks) {
    motorHomeTicks = rawTicks;
    motorCeilingTicks = ceilingTicks;
  }
  void setLimits(double minDeg, double maxDeg) { minLiftDeg = minDeg; maxLiftDeg = maxDeg; }
  void setBatteryVolts(double volts) { if (volts > 6) { batteryVolts = volts; } }
  /** Share of the top speed allowed: re-homing after belt slips slows the hood down. */
  void setSpeedScale(double scale) { speedScale = scale; }
  /** Drives a fixed power instead of the controller, cut outside [minDeg, maxDeg]; NaN returns to control. */
  void setOpenLoopPower(double p, double minDeg, double maxDeg) {
    openLoopMinDeg = minDeg;
    openLoopMaxDeg = maxDeg;
    if (Double.isNaN(p) && !Double.isNaN(openLoopPower)) {
      resetRequested = true; // Back to control: start the profile from where the hood is
    }
    openLoopPower = p;
  }

  /**
   * Takes the motor: the loop drives it from now on. mainLoopTicks is the main loop's reading of the
   * REV, with the hood at rest: the loop's own reading must match it or the loop is not used.
   */
  void activate(int mainLoopTicks) {
    if (hasFailed()) {
      return;
    }
    if (!active) {
      try {
        int fresh = readRevTicks();
        if (Math.abs(fresh - mainLoopTicks) > ACTIVATION_TOLERANCE_TICKS) {
          fail("its REV reading " + fresh + " did not match the main loop's " + mainLoopTicks);
          return;
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return;
      } catch (Exception e) {
        fail("REV read failed at start: " + e);
        return;
      }
    }
    synchronized (powerLock) {
      if (!active) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastWritten = Double.NaN;
        resetRequested = true;
        active = true;
      }
    }
    heartbeat();
    if (thread == null || !thread.isAlive()) {
      thread = new Thread(this, "HoodLoop");
      thread.setDaemon(true);
      thread.setPriority(Thread.NORM_PRIORITY + 1);
      thread.start();
    }
  }

  /** Cuts power; the loop keeps reading but writes nothing until activated again. */
  void deactivate() {
    synchronized (powerLock) {
      if (active) {
        active = false;
        motor.setPower(0);
        lastWritten = 0;
      }
    }
  }

  boolean isActive() { return active; }
  boolean hasFailed() { return !failure.isEmpty(); }
  String failure() { return failure; }
  void fail(String reason) {
    fail(reason, false);
  }
  /**
   * Recoverable: the encoders disagreed, which re-homing on the lower stop settles, after which
   * {@link #clearFailure()} returns the loop to service. Otherwise the hub's own loop takes over.
   */
  private void fail(String reason, boolean canRecover) {
    recoverable = canRecover;
    failure = reason;
    deactivate();
  }
  boolean hasRecoverableFailure() { return hasFailed() && recoverable; }
  void clearFailure() {
    recoverable = false;
    failure = "";
  }
  /** Motor ticks above the lower stop, as the loop last read them. */
  int motorTicks() { return motorTicks; }
  boolean isAtCeiling() { return atCeiling; }
  double posDeg() { return posDeg; }
  double velDegPerSec() { return velDegPerSec; }
  double power() { return power; }
  double setpointDeg() { return setpointDeg; }
  double setpointVelDegPerSec() { return setpointVel; }
  boolean isSettled() { return settled; }
  double periodMsAvg() { return periodMsAvg; }
  double periodMsMax() { return periodMsMax; }

  /** Every step since the last call: {seconds, lift, setpoint, speed, power}, each an array. */
  double[][] drainSamples() {
    synchronized (sampleLock) {
      double[][] out = new double[5][sampleCount];
      for (int k = 0; k < 5; k++) {
        System.arraycopy(samples[k], 0, out[k], 0, sampleCount);
      }
      sampleCount = 0;
      periodMsMax = 0;
      return out;
    }
  }

  // ---- Loop thread ---------------------------------------------------------

  @Override public void run() {
    long lastNs = System.nanoTime();
    double lastPos = Double.NaN;
    double vel = 0;
    double wrongWayS = 0;
    double stallS = 0;
    double ceilingS = 0;
    int errors = 0;
    try {
      while (!Thread.currentThread().isInterrupted()) {
        long startNs = System.nanoTime();
        if (startNs - heartbeatNs > HEARTBEAT_TIMEOUT_NS) {
          deactivate(); // The main loop has stopped: never leave the hood driven
          return;
        }
        if (!active) {
          lastPos = Double.NaN;
          follow.reset();
          ceilingS = 0;
          Thread.sleep(IDLE_PERIOD_MS);
          lastNs = System.nanoTime();
          continue;
        }
        int ticks;
        int motorRaw;
        try {
          ticks = readRevTicks();
          motorRaw = motor.getCurrentPosition();
          errors = 0;
        } catch (InterruptedException e) {
          throw e;
        } catch (Exception e) {
          if (++errors >= MAX_CONSECUTIVE_ERRORS) {
            fail("REV read failed: " + e);
            return;
          }
          continue;
        }
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastNs) / 1e9;
        lastNs = nowNs;
        double pos = (ticks - revZero) / ticksPerDeg;
        int motorUp = motorRaw - motorHomeTicks;
        motorTicks = motorUp;
        if (follow.update(ticks, motorRaw)) {
          fail("REV not following the motor: unplugged, loose on its shaft, or the belt skipping", true);
          return;
        }
        if (!Double.isNaN(lastPos) && dt > 0) {
          vel += VELOCITY_FILTER * ((pos - lastPos) / dt - vel);
        } else {
          vel = 0;
        }
        lastPos = pos;

        if (resetRequested) {
          resetRequested = false;
          controller.reset();
        }
        double p;
        double open = openLoopPower;
        if (!Double.isNaN(open)) {
          p = pos >= openLoopMaxDeg && open > 0 || pos <= openLoopMinDeg && open < 0 ? 0 : open;
          setpointDeg = Double.NaN;
        } else {
          HoodSubsystem.fillFastGains(gains);
          gains.maxVelDegPerSec *= speedScale;
          p = controller.update(dt, pos, vel, goalDeg, gains) * 12.0 / batteryVolts;
          setpointDeg = controller.setpointDeg();
          setpointVel = controller.setpointVelDegPerSec();
          settled = controller.isSettled();
        }
        if (Double.isNaN(open) && Math.abs(p) >= WRONG_WAY_POWER && Math.abs(vel) > WRONG_WAY_DEG_PER_SEC
            && Math.signum(p) != Math.signum(vel)) {
          wrongWayS += dt;
          if (wrongWayS >= WRONG_WAY_SECONDS) {
            fail("the hood moved against its power: REV direction wrong?");
            return;
          }
        } else {
          wrongWayS = 0;
        }
        // Reset only once the hood moves or the drive stops: the profile restarting every 50 ms dipped the
        // power below STALL_POWER, and a reset on each dip let a REV reading stuck at 0 drive on for 1.2 s
        if (!Double.isNaN(open) || p == 0 || Math.abs(vel) >= STALL_DEG_PER_SEC) {
          stallS = 0;
        } else if (Math.abs(p) >= STALL_POWER) {
          stallS += dt;
          if (stallS >= STALL_SECONDS) {
            fail("the hood did not move under power: REV reading frozen, or the hood jammed");
            return;
          }
        }
        // Never push further past either end of the allowed range
        if (pos > maxLiftDeg && p > 0 || pos < minLiftDeg && p < 0) {
          p = 0;
        }
        // The limits above go by the REV; this one by the motor, should the REV under-read
        atCeiling = motorUp >= motorCeilingTicks && p > 0;
        if (atCeiling) {
          p = 0;
          ceilingS += dt;
          if (ceilingS >= CEILING_HOLD_SECONDS) {
            fail("the motor reached its ceiling with the REV short of the goal: encoders disagree", true);
            return;
          }
        } else {
          ceilingS = 0;
        }
        p = Math.max(-1, Math.min(1, p));
        synchronized (powerLock) {
          if (active && (Double.isNaN(lastWritten) || Math.abs(p - lastWritten) > 0.002 || p == 0 && lastWritten != 0)) {
            motor.setPower(p);
            lastWritten = p;
          }
        }
        posDeg = pos;
        velDegPerSec = vel;
        power = p;
        record(nowNs / 1e9, pos, setpointDeg, vel, p);

        double periodMs = (System.nanoTime() - startNs) / 1e6;
        periodMsAvg += 0.05 * (periodMs - periodMsAvg);
        periodMsMax = Math.max(periodMsMax, periodMs);
        long sleepMs = Math.round(HoodSubsystem.FAST_PERIOD_MS - periodMs);
        if (sleepMs > 0) {
          Thread.sleep(sleepMs);
        }
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } catch (RuntimeException e) {
      fail("loop stopped: " + e);
    } finally {
      deactivate();
    }
  }

  /** The REV count as the SDK would report it, read fresh from the hub. */
  private int readRevTicks() throws Exception {
    return revSign * new LynxGetBulkInputDataCommand(revModule).sendReceive().getEncoder(revPort);
  }

  private void record(double t, double pos, double sp, double vel, double p) {
    synchronized (sampleLock) {
      if (sampleCount < SAMPLE_CAPACITY) {
        samples[0][sampleCount] = t;
        samples[1][sampleCount] = pos;
        samples[2][sampleCount] = sp;
        samples[3][sampleCount] = vel;
        samples[4][sampleCount] = p;
        sampleCount++;
      }
    }
  }
}
