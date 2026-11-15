package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.subsystems.VisionInputs;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.CameraMount;
import org.firstinspires.ftc.teamcode.vision.CameraMounts;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.Logger;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * A Robot on simulated hardware for {@link RobotLoopTraceTest}: every device is a fake that records what is written
 * to it, and simple models move the flywheels, the hood, the drive and the balls from those writes, so the aim gets
 * ready and balls leave. Once a test sets {@link #truth}, the cameras see the Hive tags and up Cells from it, through
 * the inputs the Robot reads in replay. Time is simulated: the Robot's and the subsystems' clocks are swapped for ones
 * that step {@link #DT_S} a loop, so a run is the same every time.
 */
final class RobotSim {
  static final double DT_S = 0.02;
  private static final long DT_NANOS = 20_000_000L;
  private static final String[] SHOOTERS = {HardwareNames.SHOOTER_LEFT, HardwareNames.SHOOTER_RIGHT};
  /** Motor counts per REV count on the hood's pinion (HoodEncoderGuard: 384.5 vs 8192 a turn). */
  private static final double REV_PER_MOTOR = 8192 / 384.5;
  private static final double M_PER_IN = 0.0254;

  static long simNanos;

  /** An ElapsedTime on the simulated clock. */
  static final class SimClock extends ElapsedTime {
    @Override
    protected long nsNow() {
      return simNanos;
    }
  }

  final Map<String, Map<String, Object>> props = new HashMap<>();
  /** This loop's hardware writes, telemetry and rumbles, in order. */
  final List<String> events = new ArrayList<>();
  final SimHardwareMap hardwareMap = new SimHardwareMap();
  final SimGamepad gamepad = new SimGamepad();
  final SimPinpoint pinpoint;
  final Robot robot;
  private final MotorConfigurationType motorType = new MotorConfigurationType();

  /** Where the robot really is, {x, y, heading} on the field; null keeps the cameras blind. Odometry drifts from it. */
  double[] truth;
  Cell redUp = Cell.AUDIENCE;
  Cell blueUp = Cell.SCORING;
  private double[] truthAtFrame;
  private int loopCount;
  double intakeAmps = 2.0;
  int balls = 3;
  int ballsOut;
  private int feedLoops;
  private int intakeLoops;
  private final double[] shooterVel = new double[2];
  private double hoodMotorPos;
  private double hoodMotorVel;
  private double vForward;
  private double vRight;
  private double omega;

  RobotSim() {
    simNanos = 1_000_000_000L;
    motorType.setTicksPerRev(28);
    motorType.setGearing(1);
    motorType.setMaxRPM(6000);
    motorType.setAchieveableMaxRPMFraction(0.85);
    for (String motor : new String[] {HardwareNames.DRIVE_FRONT_LEFT, HardwareNames.DRIVE_FRONT_RIGHT,
        HardwareNames.DRIVE_BACK_LEFT, HardwareNames.DRIVE_BACK_RIGHT, HardwareNames.SHOOTER_LEFT,
        HardwareNames.SHOOTER_RIGHT, HardwareNames.INTAKE_MOTOR, HardwareNames.HOOD_MOTOR}) {
      hardwareMap.devices.put(motor, device(DcMotorEx.class, motor, true));
    }
    for (String servo : new String[] {HardwareNames.INDEXER_SERVO_LEFT, HardwareNames.INDEXER_SERVO_RIGHT,
        HardwareNames.FEEDER_SERVO}) {
      hardwareMap.devices.put(servo, device(CRServo.class, servo, true));
    }
    hardwareMap.devices.put(HardwareNames.RAMP_SERVO, device(Servo.class, HardwareNames.RAMP_SERVO, true));
    hardwareMap.voltageSensor.put("Control Hub", device(VoltageSensor.class, "battery", false));
    pinpoint = new SimPinpoint(device(I2cDeviceSynchSimple.class, "pinpointBus", false));
    hardwareMap.devices.put(HardwareNames.PINPOINT, pinpoint);

    Telemetry telemetry = telemetry();
    // Replay mode leaves out the cameras and the loop watchdog's thread; inputs and outputs still log as live
    Logger.setReplay(true);
    robot = new Robot(hardwareMap, telemetry, gamepad);
    int swapped = swapClocks(robot, Collections.newSetFromMap(new IdentityHashMap<>()), 0);
    if (swapped < 5) {
      throw new IllegalStateException("only " + swapped + " clocks swapped");
    }
  }

  static void startLogger() {
    Logger.reset();
    Logger.disableConsoleCapture();
    Logger.setTimeSource(() -> simNanos * 1e-9);
    Logger.start();
  }

  static void stopLogger() {
    Logger.end();
    Logger.setReplay(false);
  }

  /** Every ElapsedTime the Robot reaches through team code, swapped for a simulated one. */
  private static int swapClocks(Object o, Set<Object> seen, int depth) {
    if (o == null || depth > 4 || !seen.add(o)) {
      return 0;
    }
    int swapped = 0;
    for (Class<?> c = o.getClass(); c != null && c.getName().startsWith("org.firstinspires.ftc.teamcode");
         c = c.getSuperclass()) {
      for (Field f : c.getDeclaredFields()) {
        if (Modifier.isStatic(f.getModifiers()) || f.getType().isPrimitive() || f.getType().isArray()) {
          continue;
        }
        try {
          f.setAccessible(true);
          Object v = f.get(o);
          if (v instanceof ElapsedTime && !(v instanceof SimClock)) {
            f.set(o, new SimClock());
            swapped++;
          } else if (v != null && v.getClass().getName().startsWith("org.firstinspires.ftc.teamcode")) {
            swapped += swapClocks(v, seen, depth + 1);
          }
        } catch (ReflectiveOperationException e) {
          throw new IllegalStateException(e);
        }
      }
    }
    return swapped;
  }

  /** One INIT loop. */
  void initLoop() {
    simNanos += DT_NANOS;
    Logger.periodicBeforeUser();
    events.clear();
    feedCameras();
    robot.initLoop();
    events.addAll(AndroidLogLines.drain());
    step();
  }

  /** One running loop, as Lebot Teleop runs it when {@code powerTelemetry}. */
  void loop(boolean powerTelemetry) {
    simNanos += DT_NANOS;
    Logger.periodicBeforeUser();
    events.clear();
    feedCameras();
    if (powerTelemetry) {
      robot.addPowerTelemetry();
    }
    robot.periodic();
    events.addAll(AndroidLogLines.drain());
    step();
  }

  /** Gamepad back to rest: nothing pressed. */
  void release() {
    Gamepad g = gamepad;
    g.left_stick_x = g.left_stick_y = g.right_stick_x = g.right_stick_y = 0;
    g.left_trigger = g.right_trigger = 0;
    g.a = g.b = g.x = g.y = false;
    g.dpad_up = g.dpad_down = g.dpad_left = g.dpad_right = false;
    g.left_bumper = g.right_bumper = g.back = g.start = false;
  }

  /** Everything logged so far (PsiKit keeps each key's last value) and the Robot's getters, by key. */
  Map<String, String> values() {
    Map<String, String> v = new TreeMap<>();
    for (Map.Entry<String, LogTable.LogValue> e : Logger.getEntry().getAll(false).entrySet()) {
      v.put(e.getKey(), format(e.getValue().getObject()));
    }
    Robot r = robot;
    v.put("get/Pose", r.getPose().xM + " " + r.getPose().yM + " " + r.getPose().headingRad);
    v.put("get/Velocity", Arrays.toString(r.getVelocity()));
    v.put("get/PoseValid", r.isPoseValid() + " known " + r.isPoseKnown() + " fixes " + r.getPositionFixes());
    v.put("get/AimCell", r.getAimCellX() + " " + r.getAimCellY());
    Aim aim = r.getLastAim();
    v.put("get/Aim", aim == null ? "null" : aim.backward + " " + aim.shot.valid + " " + aim.shot.reason + " "
        + aim.lockHeadingRad + " " + aim.lockRateRadPerSec + " " + aim.rpmNow + " " + aim.rpmRatePerSec + " "
        + aim.hoodNowDeg + " " + aim.hoodRateDegPerSec + " " + aim.headingToleranceRad);
    v.put("get/Feed", r.feedStartedThisLoop() + " " + r.ballLeftThisLoop() + " " + r.isFeederEmpty() + " "
        + r.isStackFull() + " " + r.isVolleyShot());
    v.put("get/Volley", r.getVolleyLeftAtRpm() + " " + r.getVolleyDipDepthRpm() + " " + r.volleyFeedPower() + " "
        + r.volleyIndexerPower() + " " + r.volleyBoostRpm() + " " + r.volleyFastRecovery());
    v.put("get/Cell", r.getUpCell() + " " + r.isUpCellConfirmed() + " " + r.getUpCellAgreeingFrames() + " "
        + r.getAlliance() + " " + r.driverForwardHeadingRad() + " " + r.allianceWarning());
    // Not getTagsSeenAgoS: it takes the frame time from System.nanoTime(), which the simulated frames aren't on
    v.put("get/Misc", r.getBatteryVolts() + " " + r.isTelemetryDetail() + " " + r.isOdometryReady() + " "
        + r.isDriverOnFarSide());
    v.put("sim/Balls", balls + " out " + ballsOut);
    return v;
  }

  static String format(Object o) {
    if (o instanceof double[]) return Arrays.toString((double[]) o);
    if (o instanceof float[]) return Arrays.toString((float[]) o);
    if (o instanceof long[]) return Arrays.toString((long[]) o);
    if (o instanceof boolean[]) return Arrays.toString((boolean[]) o);
    if (o instanceof byte[]) return Arrays.toString((byte[]) o);
    if (o instanceof String[]) return Arrays.toString((String[]) o);
    return String.valueOf(o);
  }

  private double power(String name) {
    Object p = props.get(name).get("Power");
    return p instanceof Double ? (Double) p : 0.0;
  }

  /** Each camera's frame, every third loop (the two a loop apart), as seen from where the robot truly was a loop ago. */
  private void feedCameras() {
    loopCount++;
    for (VisionSubsystem.Side side : VisionSubsystem.Side.values()) {
      VisionInputs in = robot.vision.getInputs(side);
      in.available = true;
      in.fps = 20;
      in.captureNanos = simNanos;
      List<Integer> ids = new ArrayList<>();
      List<String> names = new ArrayList<>();
      List<double[]> seen = new ArrayList<>();
      if (truthAtFrame != null && loopCount % 3 == side.ordinal()) {
        CameraMount mount = side == VisionSubsystem.Side.LEFT ? CameraMounts.LEFT_CAMERA : CameraMounts.RIGHT_CAMERA;
        for (int id = 30; id <= 45; id++) {
          Cell up = VisionCorrector.tagAlliance(id) == Alliance.RED ? redUp : blueUp;
          double[] p = inCamera(mount, truthAtFrame, VisionCorrector.tagFieldPosition(id, up));
          if (inView(p)) {
            ids.add(id);
            names.add("");
            seen.add(p);
          }
        }
        for (Alliance alliance : Alliance.values()) {
          Cell up = alliance == Alliance.RED ? redUp : blueUp;
          double[] aim = VisionCorrector.upCellAimPoint(alliance, up);
          int tag = alliance == Alliance.RED ? (up == Cell.SCORING ? 30 : 34) : (up == Cell.SCORING ? 42 : 38);
          double[] p = inCamera(mount, truthAtFrame,
              new double[] {aim[0], aim[1], VisionCorrector.tagFieldPosition(tag, up)[2]});
          if (inView(p)) {
            ids.add(VisionInputs.CLUSTER_ID);
            names.add(alliance.name() + " " + up.name());
            seen.add(p);
          }
        }
      }
      int n = ids.size();
      in.tagIds = new int[n];
      in.clusterNames = new String[n];
      in.range = new double[n];
      in.bearing = new double[n];
      in.elevation = new double[n];
      in.yaw = new double[n];
      in.x = new double[n];
      in.y = new double[n];
      in.z = new double[n];
      in.decisionMargin = new double[n];
      in.frameNanos = new long[n];
      for (int i = 0; i < n; i++) {
        double[] p = seen.get(i);
        in.tagIds[i] = ids.get(i);
        in.clusterNames[i] = names.get(i);
        in.x[i] = p[0] / M_PER_IN;
        in.y[i] = p[1] / M_PER_IN;
        in.z[i] = p[2] / M_PER_IN;
        in.range[i] = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) / M_PER_IN;
        in.bearing[i] = Math.toDegrees(Math.atan2(-p[0], p[1]));
        in.elevation[i] = Math.toDegrees(Math.atan2(p[2], p[1]));
        in.decisionMargin[i] = in.tagIds[i] == VisionInputs.CLUSTER_ID ? Double.NaN : 60;
        in.frameNanos[i] = simNanos - DT_NANOS;
      }
    }
  }

  private static boolean inView(double[] p) {
    return p[1] > 0.3 && Math.abs(p[0]) < 0.8 * p[1] && Math.abs(p[2]) < 0.6 * p[1]
        && Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) < 3.2;
  }

  /**
   * A field point as the camera reports it, {right, forward along the lens, up} in metres: the point's offset from
   * the camera projected on the camera's axes (as VisionCorrectorTest's observe).
   */
  private static double[] inCamera(CameraMount camera, double[] robot, double[] point) {
    double dx = point[0] - robot[0];
    double dy = point[1] - robot[1];
    double cos = Math.cos(robot[2]);
    double sin = Math.sin(robot[2]);
    double[] fromCamera = {cos * dx + sin * dy - camera.forwardM, -sin * dx + cos * dy - camera.leftM,
        point[2] - camera.heightM};
    double[] forward = {Math.cos(camera.pitchRad) * Math.cos(camera.yawRad),
        Math.cos(camera.pitchRad) * Math.sin(camera.yawRad), Math.sin(camera.pitchRad)};
    double[] right = {Math.sin(camera.yawRad), -Math.cos(camera.yawRad), 0};
    double[] up = {right[1] * forward[2] - right[2] * forward[1], right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0]};
    double[] cameraRight = new double[3];
    double[] cameraUp = new double[3];
    for (int axis = 0; axis < 3; axis++) {
      cameraRight[axis] = Math.cos(camera.rollRad) * right[axis] + Math.sin(camera.rollRad) * up[axis];
      cameraUp[axis] = -Math.sin(camera.rollRad) * right[axis] + Math.cos(camera.rollRad) * up[axis];
    }
    return new double[] {dot(fromCamera, cameraRight), dot(fromCamera, forward), dot(fromCamera, cameraUp)};
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  /** Moves the simulated robot on by one loop from what the Robot last wrote. */
  private void step() {
    // Drive: the wheels' mix (MecanumDriveSubsystem.mix) undone, then a lag to a top speed
    double fl = power(HardwareNames.DRIVE_FRONT_LEFT);
    double fr = power(HardwareNames.DRIVE_FRONT_RIGHT);
    double bl = power(HardwareNames.DRIVE_BACK_LEFT);
    double br = power(HardwareNames.DRIVE_BACK_RIGHT);
    double forward = (fl + fr + bl + br) / 4;
    double right = (fl - fr - bl + br) / 4;
    double clockwise = (fl - fr + bl - br) / 4;
    double lag = DT_S / 0.15;
    vForward += (forward * 1.8 - vForward) * lag;
    vRight += (right * 1.5 - vRight) * lag;
    omega += (-clockwise * 6.0 - omega) * lag;
    double c = Math.cos(pinpoint.h);
    double s = Math.sin(pinpoint.h);
    pinpoint.vx = vForward * c + vRight * s;
    pinpoint.vy = vForward * s - vRight * c;
    pinpoint.w = omega;
    pinpoint.x += pinpoint.vx * DT_S;
    pinpoint.y += pinpoint.vy * DT_S;
    pinpoint.h = Angles.normalize(pinpoint.h + pinpoint.w * DT_S);
    truthAtFrame = truth == null ? null : truth.clone();
    if (truth != null) {
      double tc = Math.cos(truth[2]);
      double ts = Math.sin(truth[2]);
      truth[0] += (vForward * tc + vRight * ts) * DT_S;
      truth[1] += (vForward * ts - vRight * tc) * DT_S;
      truth[2] = Angles.normalize(truth[2] + omega * DT_S);
    }

    // Flywheels: toward the commanded speed, or the speed the commanded power gives
    for (int i = 0; i < 2; i++) {
      Map<String, Object> p = props.get(SHOOTERS[i]);
      double target = "Velocity".equals(p.get("_drive")) && p.get("Velocity") instanceof Double
          ? (Double) p.get("Velocity") : power(SHOOTERS[i]) * motorType.getAchieveableMaxTicksPerSecond();
      shooterVel[i] += (target - shooterVel[i]) * Math.min(1, DT_S / 0.12);
    }

    // Hood: run to position at the commanded speed, or open loop; the lower stop is at 0
    Map<String, Object> hood = props.get(HardwareNames.HOOD_MOTOR);
    double maxStep = Math.abs(power(HardwareNames.HOOD_MOTOR)) * 2500 * DT_S;
    double next;
    if (hood.get("Mode") == DcMotor.RunMode.RUN_TO_POSITION && hood.get("TargetPosition") instanceof Integer) {
      double error = (Integer) hood.get("TargetPosition") - hoodMotorPos;
      next = hoodMotorPos + Math.max(-maxStep, Math.min(maxStep, error));
    } else {
      next = hoodMotorPos + power(HardwareNames.HOOD_MOTOR) * 2500 * DT_S;
    }
    next = Math.max(0, next);
    hoodMotorVel = (next - hoodMotorPos) / DT_S;
    hoodMotorPos = next;

    // Balls: one leaves after the feeder has pushed 4 loops with the wheels up, and dips them
    double wheels = Math.abs(shooterVel[1]);
    if (power(HardwareNames.FEEDER_SERVO) > 0.3 && balls > 0 && wheels > 600) {
      if (++feedLoops >= 4) {
        balls--;
        ballsOut++;
        feedLoops = 0;
        shooterVel[0] *= 0.85;
        shooterVel[1] *= 0.85;
      }
    } else {
      feedLoops = 0;
    }
    if (power(HardwareNames.INTAKE_MOTOR) > 0.5 && balls < 4) {
      if (++intakeLoops >= 30) {
        balls++;
        intakeLoops = 0;
      }
    } else {
      intakeLoops = 0;
    }
  }

  private double velocityOf(String name) {
    if (name.equals(HardwareNames.SHOOTER_LEFT)) return shooterVel[0];
    if (name.equals(HardwareNames.SHOOTER_RIGHT)) return shooterVel[1];
    if (name.equals(HardwareNames.HOOD_MOTOR)) return hoodMotorVel;
    return 0.0;
  }

  private int positionOf(String name) {
    if (name.equals(HardwareNames.HOOD_MOTOR)) return (int) Math.round(hoodMotorPos);
    if (name.equals(HardwareNames.HOOD_ENCODER)) return (int) Math.round(hoodMotorPos * REV_PER_MOTOR);
    return 0;
  }

  private double batteryVolts() {
    double spin = (Math.abs(shooterVel[0]) + Math.abs(shooterVel[1])) / (2 * motorType.getAchieveableMaxTicksPerSecond());
    return 12.6 - 0.9 * spin - 0.05 * intakeAmps;
  }

  /** A fake device: setters are stored and recorded, getters read them back or the models above. */
  @SuppressWarnings("unchecked")
  private <T> T device(Class<T> type, String name, boolean record) {
    Map<String, Object> p = props.computeIfAbsent(name, k -> new HashMap<>());
    return (T) Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, (proxy, m, a) -> {
      String n = m.getName();
      switch (n) {
        case "toString":
        case "getDeviceName":
        case "getConnectionInfo":
          return name;
        case "hashCode":
          return System.identityHashCode(proxy);
        case "equals":
          return proxy == a[0];
        case "getMotorType":
          return motorType;
        case "getVelocity":
          return velocityOf(name);
        case "getCurrentPosition":
          return positionOf(name);
        case "getCurrent":
          return name.equals(HardwareNames.INTAKE_MOTOR) ? intakeAmps : 0.0;
        case "getVoltage":
          return batteryVolts();
        case "getPIDFCoefficients":
          return new PIDFCoefficients();
        default:
          break;
      }
      Class<?> returns = m.getReturnType();
      if (returns == void.class) {
        if (n.startsWith("set") && a != null && a.length == 1) {
          p.put(n.substring(3), a[0]);
        }
        if (n.equals("setPower") || n.equals("setVelocity")) {
          p.put("_drive", n.substring(3));
        }
        if (record) {
          events.add(name + "." + n + (a == null ? "()" : Arrays.toString(a)));
        }
        return null;
      }
      String prop = n.startsWith("get") ? n.substring(3) : n.startsWith("is") ? n.substring(2) : null;
      Object stored = prop == null ? null : p.get(prop);
      if (stored != null && box(returns).isInstance(stored)) {
        return stored;
      }
      if (returns == String.class) {
        return name;
      }
      return defaultFor(returns);
    });
  }

  private Telemetry telemetry() {
    return (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(), new Class<?>[] {Telemetry.class},
        (proxy, m, a) -> {
          String n = m.getName();
          if (n.equals("addData") && a.length == 2) {
            events.add("tel " + a[0] + ": " + a[1]);
          } else if (n.equals("addData") && a.length == 3 && a[2] instanceof Object[]) {
            events.add("tel " + a[0] + ": " + String.format((String) a[1], (Object[]) a[2]));
          } else if (n.equals("addLine") && a != null && a.length == 1) {
            events.add("tel " + a[0]);
          } else if (n.equals("update") || n.equals("clear")) {
            events.add("tel " + n);
          } else if (n.equals("hashCode")) {
            return System.identityHashCode(proxy);
          } else if (n.equals("equals")) {
            return proxy == a[0];
          } else if (n.equals("toString")) {
            return "telemetry";
          }
          return defaultFor(m.getReturnType());
        });
  }

  private static Class<?> box(Class<?> t) {
    if (t == double.class) return Double.class;
    if (t == float.class) return Float.class;
    if (t == int.class) return Integer.class;
    if (t == long.class) return Long.class;
    if (t == boolean.class) return Boolean.class;
    if (t == short.class) return Short.class;
    if (t == byte.class) return Byte.class;
    if (t == char.class) return Character.class;
    return t;
  }

  private static Object defaultFor(Class<?> t) {
    if (t == double.class) return 0.0;
    if (t == float.class) return 0f;
    if (t == int.class) return 0;
    if (t == long.class) return 0L;
    if (t == short.class) return (short) 0;
    if (t == byte.class) return (byte) 0;
    if (t == boolean.class) return false;
    if (t == char.class) return (char) 0;
    if (t.isEnum()) return t.getEnumConstants()[0];
    return null;
  }

  /** Serves the fake devices without the SDK's lookup, which needs Android. */
  static final class SimHardwareMap extends HardwareMap {
    final Map<String, Object> devices = new HashMap<>();

    SimHardwareMap() {
      super(null, null);
    }

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
      T device = tryGet(classOrInterface, deviceName);
      if (device == null) {
        throw new IllegalArgumentException("no " + classOrInterface.getSimpleName() + " named " + deviceName);
      }
      return device;
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
      Object device = devices.get(deviceName);
      return classOrInterface.isInstance(device) ? (T) device : null;
    }
  }

  /** Gamepad 1, recording rumbles instead of queueing them. */
  final class SimGamepad extends Gamepad {
    @Override
    public void rumble(int durationMs) {
      events.add("rumble " + durationMs);
    }

    @Override
    public void rumble(double rumble1, double rumble2, int durationMs) {
      events.add("rumble " + rumble1 + " " + rumble2 + " " + durationMs);
    }

    @Override
    public void rumbleBlips(int count) {
      events.add("rumbleBlips " + count);
    }
  }

  /** The Pinpoint, reporting the simulated pose (field metres) and velocity. */
  static final class SimPinpoint extends GoBildaPinpointDriver {
    double x;
    double y;
    double h;
    double vx;
    double vy;
    double w;

    SimPinpoint(I2cDeviceSynchSimple client) {
      super(client, true);
    }

    @Override public void update() {}
    @Override public void update(ReadData data) {}
    @Override public void resetPosAndIMU() { x = 0; y = 0; h = 0; }
    @Override public void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {}
    @Override public void setEncoderResolution(GoBildaOdometryPods pods) {}
    @Override public void setEncoderResolution(double ticksPerUnit, DistanceUnit unit) {}
    @Override public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {}

    @Override
    public void setPosition(Pose2D pose) {
      x = pose.getX(DistanceUnit.METER);
      y = pose.getY(DistanceUnit.METER);
      h = pose.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public Pose2D getPosition() {
      return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, h);
    }

    @Override public double getVelX(DistanceUnit unit) { return unit.fromMeters(vx); }
    @Override public double getVelY(DistanceUnit unit) { return unit.fromMeters(vy); }
    @Override public double getHeadingVelocity(UnnormalizedAngleUnit unit) { return unit.fromRadians(w); }
    @Override public DeviceStatus getDeviceStatus() { return DeviceStatus.READY; }
  }
}
