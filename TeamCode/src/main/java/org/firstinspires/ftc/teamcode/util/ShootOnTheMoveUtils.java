package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

/**
 * Config-free maths and data types used by
 * {@link org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove}. No FTC imports, so it runs on
 * the plain JVM. Units are SI and angles are radians unless named *Deg.
 */
public final class ShootOnTheMoveUtils {
  private ShootOnTheMoveUtils() {}

  /** Immutable field pose. */
  public static final class Pose {
    public final double x;
    public final double y;
    public final double heading;

    public Pose(double x, double y, double heading) {
      this.x = x;
      this.y = y;
      this.heading = heading;
    }
  }

  /** Result of {@link #compensate}. */
  public static final class Compensation {
    public final double launchDeg;
    public final double speedMps;
    /** Yaw to add to the bearing to the goal. */
    public final double headingOffsetRad;

    Compensation(double launchDeg, double speedMps, double headingOffsetRad) {
      this.launchDeg = launchDeg;
      this.speedMps = speedMps;
      this.headingOffsetRad = headingOffsetRad;
    }
  }

  /** Wraps an angle into [-pi, pi). */
  public static double wrapAngle(double a) {
    return a - 2.0 * Math.PI * Math.floor((a + Math.PI) / (2.0 * Math.PI));
  }

  /** Rotates a robot-frame (forward, left) vector by heading into a field-frame {x, y}. */
  public static double[] rotate(double heading, double forward, double left) {
    double c = Math.cos(heading);
    double s = Math.sin(heading);
    return new double[] {c * forward - s * left, s * forward + c * left};
  }

  /**
   * Evaluates sum of c[k] * d^i * vr^j, with terms ordered i in 0..degree, j in 0..degree-i
   * (matches poly_terms() in tools/shot_sim.py).
   */
  public static double evalPoly(double[] c, int degree, double d, double vr) {
    double sum = 0;
    int k = 0;
    for (int i = 0; i <= degree; i++) {
      double di = Math.pow(d, i);
      for (int j = 0; j <= degree - i; j++) {
        sum += c[k++] * di * Math.pow(vr, j);
      }
    }
    return sum;
  }

  /**
   * Piecewise-linear lookup over ascending xs, held at the end values outside the range. NaN for
   * an empty table.
   */
  public static double interpolate(double[] xs, double[] ys, double x) {
    int n = Math.min(xs.length, ys.length);
    if (n == 0) {
      return Double.NaN;
    }
    if (n == 1 || x < xs[0]) {
      return ys[0];
    }
    if (x > xs[n - 1]) {
      return ys[n - 1];
    }
    int i = 0;
    while (i < n - 2 && x > xs[i + 1]) {
      i++;
    }
    double f = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[i] + f * (ys[i + 1] - ys[i]);
  }

  /**
   * Corrects a polynomial shot for the robot's sideways speed.
   *
   * <p>Let u point from shooter to goal and t be u rotated 90 deg CCW. The polynomials already
   * handle speed along u; they give a ball velocity relative to the robot with horizontal part
   * h0 = s cos(theta) along u and vertical part s sin(theta). Sideways speed v_t adds v_t*t to the
   * ball's ground velocity, so aim the relative horizontal velocity at h = h0*u - v_t*t instead:
   *
   * <pre>
   *   headingOffset = atan2(-v_t, h0)
   *   theta'        = atan2(s sin(theta), hypot(h0, v_t))
   *   s'            = hypot(s, v_t)
   * </pre>
   *
   * The ball then has exactly the simulated ground-frame velocity.
   *
   * @param launchDeg polynomial launch angle
   * @param exitSpeedMps polynomial exit speed
   * @param tangentialVelMps robot speed along t
   */
  public static Compensation compensate(
      double launchDeg, double exitSpeedMps, double tangentialVelMps) {
    double th = Math.toRadians(launchDeg);
    double h0 = exitSpeedMps * Math.cos(th);
    double vz = exitSpeedMps * Math.sin(th);
    double offset = Math.atan2(-tangentialVelMps, h0);
    double hMag = Math.hypot(h0, tangentialVelMps);
    double newLaunch = Math.toDegrees(Math.atan2(vz, hMag));
    double newSpeed = Math.hypot(exitSpeedMps, tangentialVelMps);
    return new Compensation(newLaunch, newSpeed, offset);
  }

  /** Median of a non-empty array. Does not modify the input. */
  public static double median(double[] a) {
    double[] c = Arrays.copyOf(a, a.length);
    Arrays.sort(c);
    int n = c.length;
    return n % 2 == 1 ? c[n / 2] : 0.5 * (c[n / 2 - 1] + c[n / 2]);
  }

  /**
   * Fixed-size ring buffer of timestamped poses; the oldest is overwritten first. Timestamps must
   * increase. Not thread safe.
   */
  public static final class PoseHistory {
    private final long[] t;
    private final double[] x;
    private final double[] y;
    private final double[] h;
    private int head; // oldest sample
    private int count;

    public PoseHistory(int capacity) {
      int n = Math.max(1, capacity);
      t = new long[n];
      x = new double[n];
      y = new double[n];
      h = new double[n];
    }

    public void add(long tNanos, Pose p) {
      int slot;
      if (count < t.length) {
        slot = (head + count) % t.length;
        count++;
      } else {
        slot = head;
        head = (head + 1) % t.length;
      }
      t[slot] = tNanos;
      x[slot] = p.x;
      y[slot] = p.y;
      h[slot] = p.heading;
    }

    public int size() {
      return count;
    }

    public void clear() {
      head = 0;
      count = 0;
    }

    /** Translates every stored sample, e.g. after a vision correction. */
    public void shift(double dx, double dy) {
      for (int k = 0; k < count; k++) {
        int i = idx(k);
        x[i] += dx;
        y[i] += dy;
      }
    }

    /**
     * Pose at tNanos, linearly interpolated (heading wrap-safe). Up to one mean sample period
     * outside the stored range returns the end sample; further out returns null.
     */
    public Pose sample(long tNanos) {
      if (count == 0) {
        return null;
      }
      long first = t[idx(0)];
      long last = t[idx(count - 1)];
      long period = count > 1 ? (last - first) / (count - 1) : 0;
      if (tNanos < first) {
        return first - tNanos <= period ? at(0) : null;
      }
      if (tNanos > last) {
        return tNanos - last <= period ? at(count - 1) : null;
      }
      for (int k = 0; k < count - 1; k++) {
        long t0 = t[idx(k)];
        long t1 = t[idx(k + 1)];
        if (tNanos <= t1) {
          if (t1 == t0) {
            return at(k + 1);
          }
          double f = (double) (tNanos - t0) / (double) (t1 - t0);
          int a = idx(k);
          int b = idx(k + 1);
          return new Pose(
              x[a] + f * (x[b] - x[a]),
              y[a] + f * (y[b] - y[a]),
              wrapAngle(h[a] + f * wrapAngle(h[b] - h[a])));
        }
      }
      return at(count - 1);
    }

    private int idx(int k) {
      return (head + k) % t.length;
    }

    private Pose at(int k) {
      int i = idx(k);
      return new Pose(x[i], y[i], h[i]);
    }
  }
}
