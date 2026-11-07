package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;

/**
 * A planned drive for SOTM Tuning: positions and speeds every {@link #DT} seconds, ending stopped, with the time the
 * ball is asked for. Planned for the Cell on the -x, -y side ({@link #CELL_X}, {@link #CELL_Y}) and mirrored onto the
 * Cell in use. Each one is placed where all of it, and where the robot would stop from any point of it, stays
 * {@link #PLAN_MARGIN_M} inside the area {@link FieldRoute} allows.
 *
 * <p>Directions are relative to the Cell as seen from the fire point: 0 toward it, 180 away, 90 to the left (across,
 * anticlockwise round the Cell), -90 to the right. Or, for a field-frame path, field angles (90 along +y): the strip
 * beside the Hive is only half a metre wide, so fast moves fit along it but not across it.
 */
final class SotmPath {
  enum Kind {
    STRAIGHT,   // Steady speed in one direction
    ARC,        // Round the Cell at a steady distance (90 left, -90 right)
    SPEED_UP,   // Fired while speeding up
    SLOW_DOWN,  // Fired while slowing down
    STOP_FIRE,  // Hard stop, fired just after
    START_FIRE, // Fired as it sets off from standing
    REVERSE,    // Sudden reversal, fired just after it
    SCURVE,     // Weaving across a straight line
    ZIGZAG,     // Sharp direction changes every 0.45 s
    CIRCLE,     // A small loop: the direction of travel keeps turning
    ERRATIC     // Random speeds and directions, changing every 0.25-0.5 s
  }

  static final double DT = 0.02;
  static final double CELL_X = -0.44; // Red SCORING's aim point, about
  static final double CELL_Y = -0.32;
  static final double PLAN_MARGIN_M = 0.05;
  static final double STEADY_ACCEL = 2.0; // Planned speeding up and slowing down, m/s^2: FieldRoute's braking
  /** Furthest off the Cell's facing a fire point is put, degrees: Shot Tuning's grid reaches 63. */
  static final double MAX_OFF_FACING_DEG = 65;
  /** Fire point search grid, metres. */
  static final double PLACE_STEP_M = 0.04;
  static final double SHOT_CHECK_STEP_S = 0.05;
  // The robot ran up to 0.2 m/s faster than a 1 m/s plan, and a window right at the table's edge never fired
  static final double SHOT_CHECK_SPEED_SCALE = 1.2;
  // A pass needs a shot for this long from its fire time (or all its window, if shorter): about three loops
  static final double SHOT_CHECK_MIN_S = 0.4;

  final Kind kind;
  final double speedMps;
  final double dirDeg;
  final int seed;
  final double aimDistM;
  final boolean fieldFrame;
  final double fireS;
  final double windowS; // The ball is asked for from fireS to fireS + windowS
  final double[] x;
  final double[] y;
  final double[] vx;
  final double[] vy;

  private SotmPath(Kind kind, double speedMps, double dirDeg, int seed, double aimDistM, boolean fieldFrame,
      double fireS, double windowS, double[] x, double[] y, double[] vx, double[] vy) {
    this.fieldFrame = fieldFrame;
    this.kind = kind;
    this.speedMps = speedMps;
    this.dirDeg = dirDeg;
    this.seed = seed;
    this.aimDistM = aimDistM;
    this.fireS = fireS;
    this.windowS = windowS;
    this.x = x;
    this.y = y;
    this.vx = vx;
    this.vy = vy;
  }

  double durationS() {
    return (x.length - 1) * DT;
  }

  /** {x, y, vx, vy} at time t, held at the ends. */
  double[] sample(double t) {
    double f = Math.max(0, Math.min(x.length - 1, t / DT));
    int i = Math.min(x.length - 2, (int) Math.floor(f));
    double a = f - i;
    return new double[] {x[i] + a * (x[i + 1] - x[i]), y[i] + a * (y[i + 1] - y[i]),
        vx[i] + a * (vx[i + 1] - vx[i]), vy[i] + a * (vy[i + 1] - vy[i])};
  }

  /** The same path for the Cell on the side given by the signs of cellX, cellY. */
  SotmPath mirroredFor(double cellX, double cellY) {
    double mx = cellX < 0 ? 1 : -1;
    double my = cellY < 0 ? 1 : -1;
    int n = x.length;
    double[] nx = new double[n];
    double[] ny = new double[n];
    double[] nvx = new double[n];
    double[] nvy = new double[n];
    for (int i = 0; i < n; i++) {
      nx[i] = x[i] * mx;
      ny[i] = y[i] * my;
      nvx[i] = vx[i] * mx;
      nvy[i] = vy[i] * my;
    }
    return new SotmPath(kind, speedMps, dirDeg, seed, aimDistM, fieldFrame, fireS, windowS, nx, ny, nvx, nvy);
  }

  String label() {
    String what = kind == Kind.ERRATIC ? kind.name() + "#" + seed
        : String.format(Locale.US, "%s %s%s", kind.name(), fieldFrame ? fieldDirName(dirDeg) : dirName(dirDeg),
            kind == Kind.CIRCLE ? (seed % 2 == 0 ? " anticlockwise" : " clockwise") : "");
    return String.format(Locale.US, "%s %.2f m/s at %.2f m", what, speedMps, aimDistM);
  }

  /** For the Cell on the -x, -y side; mirrored with the path. */
  static String fieldDirName(double deg) {
    switch ((int) Math.round(deg)) {
      case 0: return "field +x";
      case 180: return "field -x";
      case 90: return "field +y";
      case -90: return "field -y";
      default: return String.format(Locale.US, "field %.0fdeg", deg);
    }
  }

  static String dirName(double deg) {
    switch ((int) Math.round(deg)) {
      case 0: return "toward";
      case 180: return "away";
      case 90: return "left";
      case -90: return "right";
      case 45: return "toward-left";
      case -45: return "toward-right";
      case 135: return "away-left";
      case -135: return "away-right";
      default: return String.format(Locale.US, "%.0fdeg", deg);
    }
  }

  // ---- Building

  /** Desired speed in the fire point's frame {toward, left} and the acceleration allowed reaching it. */
  private interface Profile {
    double[] at(double t); // {toward, left, accel}; null once finished
  }

  /** How long each kind runs before and after its fire time, and its window. */
  private static final class Shape {
    Profile profile;
    double fireS;
    double windowS;
  }

  private static Shape shape(Kind kind, final double s, double dirDeg, int seed) {
    final double cu = Math.cos(Math.toRadians(dirDeg));
    final double cw = Math.sin(Math.toRadians(dirDeg));
    final Shape sh = new Shape();
    final double ramp = s / STEADY_ACCEL;
    switch (kind) {
      case STRAIGHT:
      case ARC: {
        // FIRE_STEADY_STICK_S and the robot settling at speed: it lags 0.3-0.5 s getting going (7 Nov, 100426)
        sh.fireS = ramp + 0.6;
        sh.windowS = 0.7;
        final double stopS = sh.fireS + sh.windowS + 0.1;
        sh.profile = t -> t > stopS + ramp + 0.3 ? null
            : t < stopS ? new double[] {s * cu, s * cw, STEADY_ACCEL} : new double[] {0, 0, STEADY_ACCEL};
        break;
      }
      case SPEED_UP: {
        final double a = 1.0;
        sh.fireS = 0.4 + 0.5 * s / a; // Halfway up to speed
        sh.windowS = 0.35;
        final double stopS = 0.4 + s / a + 0.3;
        sh.profile = t -> t > stopS + ramp + 0.3 ? null
            : t < 0.4 ? new double[] {0, 0, a}
            : t < stopS ? new double[] {s * cu, s * cw, a} : new double[] {0, 0, STEADY_ACCEL};
        break;
      }
      case SLOW_DOWN: {
        final double a = 1.0;
        final double brakeS = ramp + 0.6;
        sh.fireS = brakeS + 0.5 * s / a; // Halfway down
        sh.windowS = 0.35;
        sh.profile = t -> t > brakeS + s / a + 0.4 ? null
            : t < brakeS ? new double[] {s * cu, s * cw, STEADY_ACCEL} : new double[] {0, 0, a};
        break;
      }
      case STOP_FIRE: {
        final double a = 2.5;
        final double brakeS = ramp + 0.5;
        sh.fireS = brakeS + s / a + 0.05;
        sh.windowS = 0.8;
        sh.profile = t -> t > sh.fireS + sh.windowS + 0.2 ? null
            : t < brakeS ? new double[] {s * cu, s * cw, STEADY_ACCEL} : new double[] {0, 0, a};
        break;
      }
      case START_FIRE: {
        final double a = 2.0;
        final double goS = 0.6;
        sh.fireS = goS;
        sh.windowS = 0.5;
        final double stopS = goS + sh.windowS + 0.3;
        sh.profile = t -> t > stopS + ramp + 0.3 ? null
            : t < goS ? new double[] {0, 0, a}
            : t < stopS ? new double[] {s * cu, s * cw, a} : new double[] {0, 0, STEADY_ACCEL};
        break;
      }
      case REVERSE: {
        final double a = 4.0;
        final double revS = ramp + 0.5;
        sh.fireS = revS + 0.15;
        sh.windowS = 0.4;
        final double stopS = sh.fireS + sh.windowS + 0.2;
        sh.profile = t -> t > stopS + ramp + 0.3 ? null
            : t < revS ? new double[] {s * cu, s * cw, STEADY_ACCEL}
            : t < stopS ? new double[] {-s * cu, -s * cw, a} : new double[] {0, 0, 2.0};
        break;
      }
      case SCURVE: {
        final double base = 0.7 * s;
        final double weave = 0.7 * s;
        final double period = 1.4;
        sh.fireS = ramp + 0.9;
        sh.windowS = 0.5;
        final double stopS = sh.fireS + sh.windowS + 0.3;
        sh.profile = t -> {
          if (t > stopS + ramp + 0.3) {
            return null;
          }
          if (t >= stopS) {
            return new double[] {0, 0, STEADY_ACCEL};
          }
          double side = weave * Math.sin(2 * Math.PI * t / period);
          return new double[] {base * cu - side * cw, base * cw + side * cu, 3.0};
        };
        break;
      }
      case ZIGZAG: {
        final double turn = Math.toRadians(50);
        final double legS = 0.45;
        sh.fireS = ramp + 2 * legS + 0.1; // Just after a change
        sh.windowS = 0.35;
        final double stopS = sh.fireS + sh.windowS + 0.2;
        final double baseRad = Math.toRadians(dirDeg);
        sh.profile = t -> {
          if (t > stopS + ramp + 0.3) {
            return null;
          }
          if (t >= stopS) {
            return new double[] {0, 0, STEADY_ACCEL};
          }
          double d = baseRad + (((int) Math.floor(t / legS)) % 2 == 0 ? turn : -turn);
          return new double[] {s * Math.cos(d), s * Math.sin(d), 4.0};
        };
        break;
      }
      case CIRCLE: {
        final double radius = 0.2; // The strip is half a metre wide
        final double omega = (seed % 2 == 0 ? 1 : -1) * s / radius;
        final double baseRad = Math.toRadians(dirDeg);
        sh.fireS = ramp + Math.PI / Math.abs(omega); // Half a loop on
        sh.windowS = 0.4;
        final double stopS = sh.fireS + sh.windowS + 0.2;
        sh.profile = t -> {
          if (t > stopS + ramp + 0.3) {
            return null;
          }
          if (t >= stopS) {
            return new double[] {0, 0, STEADY_ACCEL};
          }
          double d = baseRad + omega * t;
          return new double[] {s * Math.cos(d), s * Math.sin(d), 3.0};
        };
        break;
      }
      case ERRATIC:
      default: {
        Random random = new Random(1000L + seed);
        final List<double[]> legs = new ArrayList<>(); // {end time, toward, left}
        double t = 0;
        while (t < 2.6) {
          t += 0.25 + 0.25 * random.nextDouble();
          double d = 2 * Math.PI * random.nextDouble();
          double m = s * (0.4 + 0.6 * random.nextDouble());
          legs.add(new double[] {t, m * Math.cos(d), m * Math.sin(d)});
        }
        sh.fireS = 1.3 + 0.4 * random.nextDouble();
        sh.windowS = 0.35;
        final double stopS = sh.fireS + sh.windowS + 0.2;
        sh.profile = time -> {
          if (time > stopS + ramp + 0.3) {
            return null;
          }
          if (time >= stopS) {
            return new double[] {0, 0, STEADY_ACCEL};
          }
          for (double[] leg : legs) {
            if (time < leg[0]) {
              return new double[] {leg[1], leg[2], 4.0};
            }
          }
          return new double[] {0, 0, STEADY_ACCEL};
        };
        break;
      }
    }
    return sh;
  }

  /**
   * The path firing at (fireX, fireY), or null if any of it leaves the area. ARC circles the Cell at the fire
   * point's distance; the rest keep the directions they have at the fire point.
   */
  static SotmPath build(Kind kind, double speedMps, double dirDeg, int seed, boolean fieldFrame, double fireX,
      double fireY, double cellX, double cellY) {
    return build(kind, speedMps, dirDeg, seed, fieldFrame, fireX, fireY, cellX, cellY, true);
  }

  private static SotmPath build(Kind kind, double speedMps, double dirDeg, int seed, boolean fieldFrame, double fireX,
      double fireY, double cellX, double cellY, boolean checkFit) {
    Shape sh = shape(kind, speedMps, dirDeg, seed);
    double ux = cellX - fireX;
    double uy = cellY - fireY;
    double r = Math.hypot(ux, uy);
    if (r < 1e-6) {
      return null;
    }
    ux /= r;
    uy /= r;
    if (fieldFrame) {
      ux = 1;
      uy = 0;
    }
    double wx = -uy;
    double wy = ux;
    List<double[]> pts = new ArrayList<>(); // {x, y, vx, vy}
    double px = 0;
    double py = 0;
    double vx = 0;
    double vy = 0;
    if (kind == Kind.ARC) {
      px = fireX; // Placed on the circle, turned round the Cell afterward
      py = fireY;
    }
    double side = dirDeg >= 0 ? 1 : -1;
    for (int i = 0; i < 2000; i++) {
      double t = i * DT;
      double[] want = sh.profile.at(t);
      if (want == null) {
        break;
      }
      double tx;
      double ty;
      if (kind == Kind.ARC) {
        double rx = px - cellX;
        double ry = py - cellY;
        double rr = Math.hypot(rx, ry);
        double speed = Math.hypot(want[0], want[1]);
        double tanX = -ry / rr * side; // Anticlockwise round the Cell for left
        double tanY = rx / rr * side;
        double pull = 2.0 * (r - rr); // Back onto the circle
        tx = tanX * speed + rx / rr * pull;
        ty = tanY * speed + ry / rr * pull;
      } else {
        tx = want[0] * ux + want[1] * wx;
        ty = want[0] * uy + want[1] * wy;
      }
      double dvx = tx - vx;
      double dvy = ty - vy;
      double dv = Math.hypot(dvx, dvy);
      double maxDv = want[2] * DT;
      if (dv > maxDv) {
        dvx *= maxDv / dv;
        dvy *= maxDv / dv;
      }
      pts.add(new double[] {px, py, vx, vy});
      vx += dvx;
      vy += dvy;
      px += vx * DT;
      py += vy * DT;
    }
    pts.add(new double[] {px, py, 0, 0});
    int n = pts.size();
    int fireIndex = Math.min(n - 1, (int) Math.round(sh.fireS / DT));
    double[] xs = new double[n];
    double[] ys = new double[n];
    double[] vxs = new double[n];
    double[] vys = new double[n];
    double[] atFire = pts.get(fireIndex);
    double cos = 1;
    double sin = 0;
    double dx = fireX - atFire[0];
    double dy = fireY - atFire[1];
    if (kind == Kind.ARC) { // Turned round the Cell so it fires at the fire point
      double turn = Math.atan2(fireY - cellY, fireX - cellX) - Math.atan2(atFire[1] - cellY, atFire[0] - cellX);
      cos = Math.cos(turn);
      sin = Math.sin(turn);
      dx = 0;
      dy = 0;
    }
    for (int i = 0; i < n; i++) {
      double[] p = pts.get(i);
      if (kind == Kind.ARC) {
        double rx = p[0] - cellX;
        double ry = p[1] - cellY;
        xs[i] = cellX + rx * cos - ry * sin;
        ys[i] = cellY + rx * sin + ry * cos;
        vxs[i] = p[2] * cos - p[3] * sin;
        vys[i] = p[2] * sin + p[3] * cos;
      } else {
        xs[i] = p[0] + dx;
        ys[i] = p[1] + dy;
        vxs[i] = p[2];
        vys[i] = p[3];
      }
    }
    double aimDist = Math.hypot(fireX - cellX, fireY - cellY);
    SotmPath path = new SotmPath(kind, speedMps, dirDeg, seed, aimDist, fieldFrame, sh.fireS, sh.windowS, xs, ys,
        vxs, vys);
    return !checkFit || path.fits() ? path : null;
  }

  /** Every point, and where it would stop from there, stays PLAN_MARGIN_M inside the allowed area. */
  boolean fits() {
    double m = PLAN_MARGIN_M;
    for (int i = 0; i < x.length; i++) {
      if (!FieldRoute.inBox(x[i], y[i], -m, -m)) {
        return false;
      }
      // The pass's own check (FieldRoute.headedSafely) allows SPOT_FENCE_SLACK_M past the limits; this leaves it m
      double sx = x[i] + FieldRoute.stopDistance(vx[i]);
      double sy = y[i] + FieldRoute.stopDistance(vy[i]);
      if (!FieldRoute.inBox(sx, sy, FieldRoute.SPOT_FENCE_SLACK_M - m, FieldRoute.SPOT_FENCE_SLACK_M - m)) {
        return false;
      }
    }
    return true;
  }

  /**
   * The path fired from between minDistM and maxDistM of the (canonical) Cell's aim point and no more than
   * MAX_OFF_FACING_DEG off its face (-x), as near the band's middle as fits. Null if none fits.
   */
  static SotmPath place(Kind kind, double speedMps, double dirDeg, int seed, boolean fieldFrame, double minDistM,
      double maxDistM) {
    return place(kind, speedMps, dirDeg, seed, fieldFrame, minDistM, maxDistM, false);
  }

  /** As above, also with the table having a shot (backward or hooded) through all of the fire window. */
  static SotmPath place(Kind kind, double speedMps, double dirDeg, int seed, boolean fieldFrame, double minDistM,
      double maxDistM, boolean backward) {
    List<double[]> candidates = new ArrayList<>(); // {x, y, preference}
    double mid = (minDistM + maxDistM) / 2;
    double lim = FieldRoute.FIELD_LIMIT_M - PLAN_MARGIN_M;
    for (double fx = -lim; fx <= 0; fx += PLACE_STEP_M) {
      for (double fy = -lim; fy <= lim; fy += PLACE_STEP_M) {
        double d = Math.hypot(fx - CELL_X, fy - CELL_Y);
        double off = Math.toDegrees(Math.atan2(Math.abs(fy - CELL_Y), CELL_X - fx));
        if (d < minDistM || d > maxDistM || off > MAX_OFF_FACING_DEG
            || !FieldRoute.inBox(fx, fy, -PLAN_MARGIN_M, -PLAN_MARGIN_M)) {
          continue;
        }
        candidates.add(new double[] {fx, fy, Math.abs(d - mid) + 0.002 * off});
      }
    }
    java.util.Collections.sort(candidates, (p, q) -> Double.compare(p[2], q[2]));
    if (kind == Kind.ARC) { // Its shape depends on where it is round the Cell
      for (double[] c : candidates) {
        SotmPath path = build(kind, speedMps, dirDeg, seed, fieldFrame, c[0], c[1], CELL_X, CELL_Y);
        SotmPath shootable = path == null ? null : path.trimmedToShot(backward);
        if (shootable != null) {
          return shootable;
        }
      }
      return null;
    }
    // The rest are one shape, turned to each fire point's direction to the Cell (or not, in the field frame) and
    // moved onto it: built once, checked point by point
    SotmPath shape = build(kind, speedMps, dirDeg, seed, true, 0, 0, 1e3, 0, false);
    int fireIndex = Math.min(shape.x.length - 1, (int) Math.round(shape.fireS / DT));
    double ox = shape.x[fireIndex];
    double oy = shape.y[fireIndex];
    for (double[] c : candidates) {
      double ux = 1;
      double uy = 0;
      if (!fieldFrame) {
        double d = Math.hypot(CELL_X - c[0], CELL_Y - c[1]);
        ux = (CELL_X - c[0]) / d;
        uy = (CELL_Y - c[1]) / d;
      }
      if (fitsTurned(shape, ox, oy, ux, uy, c[0], c[1])) {
        int n = shape.x.length;
        double[] xs = new double[n];
        double[] ys = new double[n];
        double[] vxs = new double[n];
        double[] vys = new double[n];
        for (int i = 0; i < n; i++) {
          double lx = shape.x[i] - ox;
          double ly = shape.y[i] - oy;
          xs[i] = c[0] + lx * ux - ly * uy;
          ys[i] = c[1] + lx * uy + ly * ux;
          vxs[i] = shape.vx[i] * ux - shape.vy[i] * uy;
          vys[i] = shape.vx[i] * uy + shape.vy[i] * ux;
        }
        SotmPath path = new SotmPath(kind, speedMps, dirDeg, seed, Math.hypot(c[0] - CELL_X, c[1] - CELL_Y),
            fieldFrame, shape.fireS, shape.windowS, xs, ys, vxs, vys);
        SotmPath shootable = path.trimmedToShot(backward);
        if (shootable != null) {
          return shootable;
        }
      }
    }
    return null;
  }

  /**
   * This path with its window cut to where the table has a shot, at the planned speed and SHOT_CHECK_SPEED_SCALE
   * times it; null if that leaves less than SHOT_CHECK_MIN_S (or the whole window, if shorter). A line past the
   * Cell at 1 m/s (7 Nov, 083925) had a shot for the first 0.25 s of its 0.7 s window and backed out of the table
   * for the rest: three runs, no ball.
   */
  SotmPath trimmedToShot(boolean backward) {
    double shotS = shotS(backward);
    if (shotS + 1e-9 < Math.min(windowS, SHOT_CHECK_MIN_S)) {
      return null;
    }
    return shotS >= windowS - 1e-9 ? this
        : new SotmPath(kind, speedMps, dirDeg, seed, aimDistM, fieldFrame, fireS, shotS, x, y, vx, vy);
  }

  /** How long from fireS, up to the end of the window, the table has a shot all the way. */
  double shotS(boolean backward) {
    double last = -1;
    for (int i = 0; ; i++) {
      double dt = Math.min(windowS, i * SHOT_CHECK_STEP_S);
      double[] s = sample(fireS + dt);
      if (!shotFrom(s, 1, backward) || !shotFrom(s, SHOT_CHECK_SPEED_SCALE, backward)) {
        return Math.max(0, last);
      }
      last = dt;
      if (dt >= windowS) {
        return windowS;
      }
    }
  }

  /** Whether the robot at {x, y, vx, vy}, its speed scaled, facing the way the shot points it, has a shot. */
  private static boolean shotFrom(double[] s, double speedScale, boolean backward) {
    double vx = s[2] * speedScale;
    double vy = s[3] * speedScale;
    Pose pose = new Pose(s[0], s[1], Math.atan2(CELL_Y - s[1], CELL_X - s[0]));
    Aim aim = null;
    for (int i = 0; i < 2; i++) { // The shooter sits off the middle, so where it points moves the shot a little
      aim = backward ? ShootOnTheMove.aimBackward(pose, vx, vy, CELL_X, CELL_Y)
          : ShootOnTheMove.aimHooded(pose, vx, vy, CELL_X, CELL_Y);
      pose = new Pose(s[0], s[1], aim.lockHeadingRad);
    }
    return aim.shot.valid;
  }

  /** {@link #fits} for the shape turned by (ux, uy) and moved so its point (ox, oy) is at (fx, fy). */
  private static boolean fitsTurned(SotmPath shape, double ox, double oy, double ux, double uy, double fx, double fy) {
    double m = PLAN_MARGIN_M;
    for (int i = 0; i < shape.x.length; i++) {
      double lx = shape.x[i] - ox;
      double ly = shape.y[i] - oy;
      double x = fx + lx * ux - ly * uy;
      double y = fy + lx * uy + ly * ux;
      if (!FieldRoute.inBox(x, y, -m, -m)) {
        return false;
      }
      double vx = shape.vx[i] * ux - shape.vy[i] * uy;
      double vy = shape.vx[i] * uy + shape.vy[i] * ux;
      if (!FieldRoute.inBox(x + FieldRoute.stopDistance(vx), y + FieldRoute.stopDistance(vy),
          FieldRoute.SPOT_FENCE_SLACK_M - m, FieldRoute.SPOT_FENCE_SLACK_M - m)) {
        return false;
      }
    }
    return true;
  }

  // ---- The plan

  /** One planned pass: its key in the CSV, the path for the canonical Cell, and the fire gates it runs under. */
  static final class Entry {
    final String key;
    final SotmPath path;
    final boolean relaxedGates; // Steady-motion gate off: the point of the pass is to fire mid-change

    Entry(String key, SotmPath path, boolean relaxedGates) {
      this.key = key;
      this.path = path;
      this.relaxedGates = relaxedGates;
    }
  }

  /** Steady passes, one speed along a line or an arc, are tuned as spots (SOTM Tuning searches each). */
  static boolean isSpot(Entry e) {
    return e.path.kind == Kind.STRAIGHT || e.path.kind == Kind.ARC;
  }

  /** At the fire point, for the canonical Cell: {speed toward the Cell, sideways speed (left +), distance}. */
  static double[] fireState(SotmPath p) {
    double[] f = p.sample(p.fireS);
    double d = Math.max(1e-6, Math.hypot(CELL_X - f[0], CELL_Y - f[1]));
    double ux = (CELL_X - f[0]) / d;
    double uy = (CELL_Y - f[1]) / d;
    return new double[] {f[2] * ux + f[3] * uy, -f[2] * uy + f[3] * ux, d};
  }

  /**
   * The spots first, in an order that spreads them over speed toward, sideways speed and distance as early as it
   * can (each next is the furthest from those before it), so a fit after the first dozen already spans them all;
   * then the other passes in plan order.
   */
  static List<Entry> spotsFirst(List<Entry> plan) {
    List<Entry> left = new ArrayList<>();
    List<double[]> state = new ArrayList<>();
    for (Entry e : plan) {
      if (isSpot(e)) {
        left.add(e);
        state.add(fireState(e.path));
      }
    }
    double[] dists = new double[state.size()];
    for (int i = 0; i < dists.length; i++) {
      dists[i] = state.get(i)[2];
    }
    Arrays.sort(dists);
    double middle = dists.length == 0 ? 0 : dists[dists.length / 2];
    List<Entry> out = new ArrayList<>();
    List<double[]> chosen = new ArrayList<>();
    while (!left.isEmpty()) {
      int best = 0;
      double bestScore = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < left.size(); i++) {
        double[] s = state.get(i);
        double score;
        if (chosen.isEmpty()) { // The slowest at the middle distance first
          score = -Math.hypot(s[0], s[1]) - Math.abs(s[2] - middle);
        } else {
          score = Double.POSITIVE_INFINITY;
          for (double[] c : chosen) {
            score = Math.min(score, Math.hypot(Math.hypot((s[0] - c[0]) / 0.4, (s[1] - c[1]) / 0.4),
                (s[2] - c[2]) / 0.25));
          }
        }
        if (score > bestScore) {
          bestScore = score;
          best = i;
        }
      }
      out.add(left.remove(best));
      chosen.add(state.remove(best));
    }
    for (Entry e : plan) {
      if (!isSpot(e)) {
        out.add(e);
      }
    }
    return out;
  }

  /** Kinds fired mid-change, which the match's steady-motion gate would hold back. */
  static boolean firesMidChange(Kind kind) {
    return kind != Kind.STRAIGHT && kind != Kind.ARC && kind != Kind.STOP_FIRE;
  }

  /**
   * Every pass for the hooded or the backward shot, most useful first, leaving out those that fit nowhere: steady
   * lines along the strip at five speeds, then Cell-relative lines and arcs, then the changes (speeding up, slowing
   * down, stops, starts, reversals, weaves, zigzags, loops, random), in each distance band.
   */
  static List<Entry> plan(boolean backward) {
    LEFT_OUT.clear();
    List<Entry> out = new ArrayList<>();
    double[] along = {90, -90};
    double[] relative = {90, -90, 0, 180};
    if (backward) {
      double[][] bands = {{0.55, 0.7}, {0.7, 0.85}};
      for (double[] band : bands) {
        for (double s : new double[] {0.15, 0.25}) {
          for (double d : along) {
            add(out, "B", Kind.STRAIGHT, s, d, 0, true, band);
          }
          for (double d : relative) {
            add(out, "B", Kind.STRAIGHT, s, d, 0, false, band);
          }
          for (double d : along) {
            add(out, "B", Kind.ARC, s, d, 0, false, band);
          }
        }
        for (double d : along) {
          add(out, "B", Kind.STOP_FIRE, 0.25, d, 0, true, band);
          add(out, "B", Kind.START_FIRE, 0.25, d, 0, true, band);
          add(out, "B", Kind.SLOW_DOWN, 0.25, d, 0, true, band);
          add(out, "B", Kind.REVERSE, 0.25, d, 0, true, band);
        }
      }
      return out;
    }
    double[][] bands = {{1.1, 1.35}, {0.9, 1.1}, {1.35, 1.75}};
    for (int round = 0; round < bands.length; round++) {
      double[] band = bands[round];
      for (double s : new double[] {0.3, 0.5, 0.7, 0.9, 1.0}) {
        for (double d : along) {
          add(out, "F", Kind.STRAIGHT, s, d, 0, true, band);
        }
      }
      for (double s : new double[] {0.3, 0.6, 0.9}) {
        for (double d : relative) {
          add(out, "F", Kind.STRAIGHT, s, d, 0, false, band);
        }
        for (double d : along) {
          add(out, "F", Kind.ARC, s, d, 0, false, band);
        }
      }
      for (double s : new double[] {0.5, 0.8}) {
        for (double d : new double[] {45, -45, 135, -135}) {
          add(out, "F", Kind.STRAIGHT, s, d, 0, false, band);
        }
      }
      for (double d : along) {
        add(out, "F", Kind.SPEED_UP, 0.8, d, 0, true, band);
        add(out, "F", Kind.SLOW_DOWN, 0.8, d, 0, true, band);
        add(out, "F", Kind.STOP_FIRE, 0.7, d, 0, true, band);
        add(out, "F", Kind.START_FIRE, 0.7, d, 0, true, band);
      }
      for (double d : relative) {
        add(out, "F", Kind.SPEED_UP, 0.6, d, 0, false, band);
        add(out, "F", Kind.SLOW_DOWN, 0.6, d, 0, false, band);
        add(out, "F", Kind.STOP_FIRE, 0.5, d, 0, false, band);
        add(out, "F", Kind.START_FIRE, 0.5, d, 0, false, band);
      }
      for (double s : new double[] {0.5, 0.8}) {
        for (double d : along) {
          add(out, "F", Kind.REVERSE, s, d, 0, true, band);
        }
        for (double d : relative) {
          add(out, "F", Kind.REVERSE, s, d, 0, false, band);
        }
      }
      for (double s : new double[] {0.6, 0.9}) {
        for (double d : along) {
          add(out, "F", Kind.SCURVE, s, d, 0, true, band);
          add(out, "F", Kind.ZIGZAG, s, d, 0, true, band);
        }
      }
      for (double s : new double[] {0.4, 0.6}) {
        add(out, "F", Kind.CIRCLE, s, 90, 0, true, band);
        add(out, "F", Kind.CIRCLE, s, -90, 1, true, band);
      }
      for (int seed = 1; seed <= 6; seed++) {
        add(out, "F", Kind.ERRATIC, seed <= 3 ? 0.6 : 0.9, 0, seed + 10 * round, true, band);
      }
    }
    return out;
  }

  private static void add(List<Entry> out, String mode, Kind kind, double s, double d, int seed, boolean fieldFrame,
      double[] band) {
    // ERRATIC tries further seeds until one fits; the key keeps the seed it used
    for (int attempt = 0; attempt < (kind == Kind.ERRATIC ? 20 : 1); attempt++) {
      SotmPath path = place(kind, s, d, seed + 100 * attempt, fieldFrame, band[0], band[1], mode.equals("B"));
      if (path != null) {
        out.add(new Entry(mode + " " + path.label(), path, firesMidChange(kind)));
        return;
      }
    }
    LEFT_OUT.add(String.format(Locale.US, "%s %s %s %.2f m/s at %.2f-%.2f m", mode, kind,
        fieldFrame ? fieldDirName(d) : dirName(d), s, band[0], band[1]));
  }

  /** Passes {@link #plan} left out for not fitting anywhere, for the tests to show. */
  static final List<String> LEFT_OUT = new ArrayList<>();
}
