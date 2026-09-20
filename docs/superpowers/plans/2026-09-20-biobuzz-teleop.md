# BIOBUZZ TeleOp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Driver-controlled robot code for the BIOBUZZ WA Qualifier: mecanum drive with Pinpoint heading, intake, indexer, twin-flywheel shooter with closed-loop velocity, and AprilTag aim assist on the alliance's Hive cell.

**Architecture:** FTCLib command-based layout already in the repo. One `OpMode` (`Main`) owns a `Robot`, which builds one `SubsystemBase` per mechanism, runs the `CommandScheduler` each loop, and polls a `GamepadEx`. Pure maths (mecanum mixing, RPM conversion) lives in static classes with JVM unit tests. Tuning values and config names live in `Constants`.

**Tech Stack:** FTC SDK v12.0, FTCLib 2.1.1 (core), Java 8, Gradle 9.1 / AGP 8.13.2, JUnit 4 for JVM tests, goBILDA Pinpoint driver (built into the SDK), VisionPortal + AprilTagProcessor.

**Spec:** `docs/superpowers/specs/2026-09-20-biobuzz-teleop-design.md`

## Global Constraints

- SDK v12.0 (`org.firstinspires.ftc:*:12.0.0`), the BIOBUZZ season release. No new SDK-external dependencies beyond JUnit 4 for tests.
- FTCLib stays at `org.ftclib.ftclib:core:2.1.1`. Drop `org.ftclib.ftclib:vision:2.1.0`; nothing uses it.
- Java 8 source level (`build.common.gradle`, do not edit).
- Config names exactly as in the spec table: `front_left_drive`, `front_right_drive`, `back_left_drive`, `back_right_drive`, `intake`, `indexer`, `shooter_left`, `shooter_right`, `pinpoint`, `Webcam 1`.
- One driver on gamepad1. Controls exactly as the spec table.
- No FTC Dashboard or other Wi-Fi tools (manual R704). Tuning goes through Driver Station telemetry.
- Commit prefixes used in this repo: `feature:`, `docs:`, `chore:`.
- Building needs JDK 17 and the Android SDK packages listed in the README. CI (`.github/workflows/build.yml`) has them. If `./gradlew` fails locally with a missing SDK or wrong JDK, follow the README "Requirements" section or push a branch and read the CI result instead.
- Anything that touches hardware is verified on the robot using the checklist in the spec, and the PR template's "Tested on the robot" section records it.

---

## File map

Create:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`: config names, motor directions, tuning values.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Alliance.java`: `enum Alliance { RED, BLUE }`.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumKinematics.java`: pure maths for field-centric rotation and wheel mixing.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShooterMath.java`: RPM to ticks-per-second and back.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeSubsystem.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IndexerSubsystem.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/VisionSubsystem.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/MecanumKinematicsTest.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShooterMathTest.java`

Modify:
- `build.dependencies.gradle`: SDK 11.2.1 to 12.0.0.
- `FtcRobotController/src/main/AndroidManifest.xml`: versionName 12.0.
- Six AprilTag sample files under `FtcRobotController/.../external/samples/`: replace with v12.0 copies.
- `TeamCode/build.gradle`: drop ftclib vision, add JUnit.
- `.github/workflows/build.yml`: run unit tests.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Main.java`: add `init_loop()` and `start()`.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`: build new subsystems, bulk caching, `GamepadEx` controls, telemetry.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDriveSubsystem.java`: use `Constants` and `MecanumKinematics`.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/OdometrySubsystem.java`: use `Constants` for the config name and pod offsets.
- `README.md`: SDK version, config-name table, on-robot checklist pointer.

---

### Task 1: Move the SDK to v12.0

**Files:**
- Modify: `build.dependencies.gradle`
- Modify: `FtcRobotController/src/main/AndroidManifest.xml:5`
- Modify: `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/{ConceptAprilTag,ConceptAprilTagEasy,ConceptAprilTagLocalization,ConceptAprilTagSwitchableCameras,RobotAutoDriveToAprilTagOmni,RobotAutoDriveToAprilTagTank}.java`
- Modify: `TeamCode/build.gradle:28-30`
- Modify: `README.md` (Requirements section)

**Interfaces:**
- Produces: the v12 AprilTag classes used in Task 7: `AprilTagDetection` (abstract, has `ftcPose.bearing`), `AprilTagSingleDetection` (`id`, `metadata`), `AprilTagClusterDetection` (`percentClusterFound`, `metadata.name`), `AprilTagGameDatabase.getCurrentGameTagLibrary()`.

- [ ] **Step 1: Bump the SDK artifact versions**

In `build.dependencies.gradle` change every `11.2.1` to `12.0.0`:

```bash
sed -i 's/:11\.2\.1/:12.0.0/g' build.dependencies.gradle
grep -c '12.0.0' build.dependencies.gradle   # expect 8
```

- [ ] **Step 2: Bump the manifest version name**

```bash
sed -i 's/android:versionName="11.2.1"/android:versionName="12.0"/' FtcRobotController/src/main/AndroidManifest.xml
grep versionName FtcRobotController/src/main/AndroidManifest.xml
```

Expected: `android:versionName="12.0">`. Leave `versionCode` as it is; v12.0 upstream did not change it.

- [ ] **Step 3: Replace the six AprilTag samples with the v12.0 copies**

The v12 cluster API changed these samples. Fetch them straight from the upstream tag so they match the library:

```bash
D=FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples
for f in ConceptAprilTag ConceptAprilTagEasy ConceptAprilTagLocalization ConceptAprilTagSwitchableCameras RobotAutoDriveToAprilTagOmni RobotAutoDriveToAprilTagTank; do
  gh api "repos/FIRST-Tech-Challenge/FtcRobotController/contents/$D/$f.java?ref=v12.0" --jq .content | base64 -d > "$D/$f.java"
done
grep -l "AprilTagSingleDetection" $D/ConceptAprilTag.java   # expect a hit
```

- [ ] **Step 4: Drop the unused FTCLib vision artifact and add JUnit**

Edit the `dependencies` block at the bottom of `TeamCode/build.gradle` to read:

```groovy
dependencies {
    implementation project(':FtcRobotController')
    implementation 'org.ftclib.ftclib:core:2.1.1'
    testImplementation 'junit:junit:4.13.2'
}
```

- [ ] **Step 5: Update the README requirement line**

In `README.md`, change

```
- **FIRST Tech Challenge SDK v11.2.1** <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v11.2.1> (already in this repository)
```

to

```
- **FIRST Tech Challenge SDK v12.0** <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0> (already in this repository, the BIOBUZZ season release)
```

- [ ] **Step 6: Build**

```bash
./gradlew :TeamCode:assembleDebug --no-daemon
```

Expected: `BUILD SUCCESSFUL`. If it fails inside FTCLib with a missing class, stop and report; that is the FTCLib-compatibility risk from the spec and needs a decision before continuing.

- [ ] **Step 7: Commit**

```bash
git add build.dependencies.gradle FtcRobotController/src/main/AndroidManifest.xml FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples TeamCode/build.gradle README.md
git commit -m "chore: move to FTC SDK v12.0, the BIOBUZZ season release"
```

---

### Task 2: Mecanum maths as a tested pure function

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumKinematics.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/MecanumKinematicsTest.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDriveSubsystem.java`
- Modify: `.github/workflows/build.yml`

**Interfaces:**
- Produces: `static double[] MecanumKinematics.toRobotRelative(double forward, double right, double headingRadians)` returning `{forward, right}`; `static double[] MecanumKinematics.mix(double forward, double right, double rotate)` returning `{frontLeft, frontRight, backLeft, backRight}` normalised so no entry exceeds 1 in magnitude.

- [ ] **Step 1: Write the failing tests**

```java
package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

public class MecanumKinematicsTest {
    private static final double EPS = 1e-9;

    @Test
    public void forwardDrivesAllWheelsForward() {
        assertArrayEquals(new double[]{1, 1, 1, 1}, MecanumKinematics.mix(1, 0, 0), EPS);
    }

    @Test
    public void strafeRightUsesDiagonalPattern() {
        assertArrayEquals(new double[]{1, -1, -1, 1}, MecanumKinematics.mix(0, 1, 0), EPS);
    }

    @Test
    public void rotateClockwiseDrivesLeftForwardRightBack() {
        assertArrayEquals(new double[]{1, -1, 1, -1}, MecanumKinematics.mix(0, 0, 1), EPS);
    }

    @Test
    public void saturationScalesAllWheelsTogether() {
        // raw = {3, -1, 1, 1}; divide by 3
        assertArrayEquals(new double[]{1, -1.0 / 3, 1.0 / 3, 1.0 / 3}, MecanumKinematics.mix(1, 1, 1), EPS);
    }

    @Test
    public void smallInputsAreNotScaledUp() {
        assertArrayEquals(new double[]{0.5, 0.5, 0.5, 0.5}, MecanumKinematics.mix(0.5, 0, 0), EPS);
    }

    @Test
    public void zeroHeadingLeavesInputUnchanged() {
        assertArrayEquals(new double[]{0.3, -0.4}, MecanumKinematics.toRobotRelative(0.3, -0.4, 0), EPS);
    }

    @Test
    public void robotFacingLeftMovesRightToGoFieldForward() {
        // Robot rotated 90 degrees anticlockwise. A field-forward command becomes robot-right.
        assertArrayEquals(new double[]{0, 1}, MecanumKinematics.toRobotRelative(1, 0, Math.PI / 2), EPS);
    }
}
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*MecanumKinematicsTest' --no-daemon
```

Expected: compile failure, `cannot find symbol MecanumKinematics`.

- [ ] **Step 3: Implement**

```java
package org.firstinspires.ftc.teamcode;

/** Pure maths for a four-wheel mecanum base. No hardware, so it runs in JVM unit tests. */
public final class MecanumKinematics {
    private MecanumKinematics() {}

    /**
     * Rotates a field-relative (forward, right) command into the robot frame.
     *
     * @param headingRadians robot heading, anticlockwise positive, 0 = facing field-forward
     * @return {forward, right} in the robot frame
     */
    public static double[] toRobotRelative(double forward, double right, double headingRadians) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        // Standard rotation by -heading with x = right, y = forward.
        return new double[]{
                forward * cos - right * sin,
                right * cos + forward * sin,
        };
    }

    /**
     * Mixes forward, strafe-right and clockwise rotation into wheel powers.
     *
     * @return {frontLeft, frontRight, backLeft, backRight}, scaled so the largest magnitude is at most 1
     */
    public static double[] mix(double forward, double right, double rotate) {
        double[] p = {
                forward + right + rotate,
                forward - right - rotate,
                forward - right + rotate,
                forward + right - rotate,
        };
        double max = 1.0;
        for (double v : p) max = Math.max(max, Math.abs(v));
        for (int i = 0; i < p.length; i++) p[i] /= max;
        return p;
    }
}
```

- [ ] **Step 4: Run the tests**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*MecanumKinematicsTest' --no-daemon
```

Expected: `BUILD SUCCESSFUL`, 7 tests passed. If `robotFacingLeftMovesRightToGoFieldForward` fails with the signs swapped, the rotation direction is wrong: the existing subsystem's `atan2`/`hypot` code is the reference behaviour, and this function must match it.

- [ ] **Step 5: Use it in the drive subsystem**

Replace the body of `drive(...)` in `MecanumDriveSubsystem.java` with:

```java
    public void drive(double forward, double right, double rotate, boolean fieldCentric, double headingRadians) {
        if (fieldCentric) {
            double[] rr = MecanumKinematics.toRobotRelative(forward, right, headingRadians);
            forward = rr[0];
            right = rr[1];
        }
        double[] p = MecanumKinematics.mix(forward, right, rotate);
        frontLeftDrive.setPower(p[0]);
        frontRightDrive.setPower(p[1]);
        backLeftDrive.setPower(p[2]);
        backRightDrive.setPower(p[3]);
    }
```

Add `import org.firstinspires.ftc.teamcode.MecanumKinematics;` and remove the now-unused `import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;`.

- [ ] **Step 6: Run tests in CI**

In `.github/workflows/build.yml`, after the "Assemble the TeamCode debug APK" step add:

```yaml
      - name: Run the TeamCode JVM unit tests
        run: ./gradlew :TeamCode:testDebugUnitTest --no-daemon --stacktrace
```

- [ ] **Step 7: Build and test**

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 8: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumKinematics.java TeamCode/src/test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDriveSubsystem.java .github/workflows/build.yml
git commit -m "feature: mecanum maths as a unit-tested pure function"
```

---

### Task 3: Constants, bulk reads and gamepad wrapper

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Alliance.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDriveSubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/OdometrySubsystem.java`

**Interfaces:**
- Produces: every name in `Constants` below, used by Tasks 4 to 8. `Robot` holds `private final GamepadEx driver` and calls `driver.readButtons()` once per loop.

- [ ] **Step 1: Write `Constants.java`**

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Hardware names, directions and tuning values. One home so drivers and Pedro Pathing can find them. */
public final class Constants {
    private Constants() {}

    // Robot Controller configuration names.
    public static final String FRONT_LEFT_DRIVE = "front_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_LEFT_DRIVE = "back_left_drive";
    public static final String BACK_RIGHT_DRIVE = "back_right_drive";
    public static final String INTAKE = "intake";
    public static final String INDEXER = "indexer";
    public static final String SHOOTER_LEFT = "shooter_left";
    public static final String SHOOTER_RIGHT = "shooter_right";
    public static final String PINPOINT = "pinpoint";
    public static final String WEBCAM = "Webcam 1";

    // Motor directions. Left side reversed so positive power on every drive motor goes forward.
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Pinpoint pod offsets from the tracking point, mm. X pod: left of centre positive.
    // Y pod: forward of centre positive. Measure on the robot per the goBILDA setup guide.
    public static final double PINPOINT_X_OFFSET_MM = 0.0;
    public static final double PINPOINT_Y_OFFSET_MM = 0.0;

    // Intake and indexer open-loop powers.
    public static final double INTAKE_POWER = 1.0;
    public static final double INDEXER_POWER = 1.0;

    // Shooter. 1:1 Yellow Jacket, 28 ticks per rev, 6000 RPM free speed.
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_SETPOINT_RPM = 3500.0;   // tune on the robot
    public static final double SHOOTER_TOLERANCE_RPM = 100.0;
    // Velocity PIDF starting point: F = 32767 / max ticks per second, P = 0.1 F, I = 0.1 P, D = 0.
    public static final double SHOOTER_P = 1.17;
    public static final double SHOOTER_I = 0.117;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 11.7;

    // Driver controls.
    public static final double TRIGGER_THRESHOLD = 0.2;

    // Aim assist: rotation = -AIM_KP * bearingDeg, clamped. Bearing is positive to the left.
    public static final double AIM_KP = 0.02;
    public static final double AIM_MAX_ROTATE = 0.5;
    public static final double AIM_DEADBAND_DEG = 1.0;
}
```

- [ ] **Step 2: Write `Alliance.java`**

```java
package org.firstinspires.ftc.teamcode;

public enum Alliance { RED, BLUE }
```

- [ ] **Step 3: Point the existing subsystems at `Constants`**

In `MecanumDriveSubsystem.java` replace the constructor body with:

```java
        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_DRIVE);

        frontLeftDrive.setDirection(Constants.FRONT_LEFT_DIRECTION);
        frontRightDrive.setDirection(Constants.FRONT_RIGHT_DIRECTION);
        backLeftDrive.setDirection(Constants.BACK_LEFT_DIRECTION);
        backRightDrive.setDirection(Constants.BACK_RIGHT_DIRECTION);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
```

and add `import org.firstinspires.ftc.teamcode.Constants;`.

In `OdometrySubsystem.java` change the two lines

```java
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
```
```java
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
```

to

```java
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT);
```
```java
        pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET_MM, Constants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
```

add `import org.firstinspires.ftc.teamcode.Constants;`, and delete the `// TODO: measure the pods' offsets ...` comment (the note now lives on the constants).

- [ ] **Step 4: Rewrite `Robot.java` with bulk caching and `GamepadEx`**

Replace the whole file:

```java
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class Robot {
    private final Telemetry telemetry;
    private final GamepadEx driver;

    public final MecanumDriveSubsystem drive;
    public final OdometrySubsystem odometry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverGamepad) {
        this.telemetry = telemetry;
        this.driver = new GamepadEx(driverGamepad);

        // One bulk read per hub per loop instead of one bus transaction per motor read.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new OdometrySubsystem(hardwareMap);
    }

    /** Called once when the OpMode enters INIT. */
    public void init() {
        odometry.init();
    }

    /** Called repeatedly while the OpMode is running. */
    public void periodic() {
        driver.readButtons();
        CommandScheduler.getInstance().run();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            odometry.resetHeading();
        }

        // Hold left bumper to drive robot-relative; otherwise drive field-relative.
        boolean fieldCentric = !driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
        drive.drive(driver.getLeftY(), driver.getLeftX(), driver.getRightX(),
                fieldCentric, odometry.getPose().getHeading(AngleUnit.RADIANS));

        telemetry.addData("Pose", odometry.getPose());
        telemetry.update();
    }

    /** Called once when the OpMode stops. */
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
```

Note `GamepadEx.getLeftY()` already negates the raw stick, so the old `-driver.left_stick_y` becomes `driver.getLeftY()`.

- [ ] **Step 5: Build and test**

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 6: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git commit -m "feature: constants class, hub bulk reads and GamepadEx controls"
```

---

### Task 4: Intake and indexer subsystems

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeSubsystem.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IndexerSubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`

**Interfaces:**
- Consumes: `Constants.INTAKE`, `INDEXER`, `INTAKE_DIRECTION`, `INDEXER_DIRECTION`, `INTAKE_POWER`, `INDEXER_POWER`, `TRIGGER_THRESHOLD`.
- Produces: `IntakeSubsystem.run()`, `reverse()`, `stop()`; `IndexerSubsystem.feed()`, `reverse()`, `stop()`. Task 6 adds the fire condition around `indexer.feed()`.

- [ ] **Step 1: Write `IntakeSubsystem.java`**

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Front compliant-wheel roller. Open loop. */
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.INTAKE);
        motor.setDirection(Constants.INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void run() {
        motor.setPower(Constants.INTAKE_POWER);
    }

    public void reverse() {
        motor.setPower(-Constants.INTAKE_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}
```

- [ ] **Step 2: Write `IndexerSubsystem.java`**

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Inclined compliant-wheel ramp that carries balls to the shooter. Brakes so held balls stay put. */
public class IndexerSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.INDEXER);
        motor.setDirection(Constants.INDEXER_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void feed() {
        motor.setPower(Constants.INDEXER_POWER);
    }

    public void reverse() {
        motor.setPower(-Constants.INDEXER_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}
```

- [ ] **Step 3: Wire them into `Robot.java`**

Add imports:

```java
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
```

Add fields after `odometry`:

```java
    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
```

Construct them after `odometry = ...`:

```java
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
```

In `periodic()`, after the drive call and before telemetry, add:

```java
        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else {
            intake.stop();
            indexer.stop();
        }
```

- [ ] **Step 4: Build**

```bash
./gradlew :TeamCode:assembleDebug --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git commit -m "feature: intake and indexer on the triggers"
```

---

### Task 5: Shooter maths, tested

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShooterMath.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShooterMathTest.java`

**Interfaces:**
- Produces: `static double ShooterMath.rpmToTicksPerSecond(double rpm, double ticksPerRev)`, `static double ShooterMath.ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev)`.

- [ ] **Step 1: Write the failing tests**

```java
package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ShooterMathTest {
    @Test
    public void freeSpeedOfOneToOneMotorIs2800TicksPerSecond() {
        assertEquals(2800.0, ShooterMath.rpmToTicksPerSecond(6000, 28), 1e-9);
    }

    @Test
    public void conversionsRoundTrip() {
        double rpm = 3500;
        double tps = ShooterMath.rpmToTicksPerSecond(rpm, 28);
        assertEquals(rpm, ShooterMath.ticksPerSecondToRpm(tps, 28), 1e-9);
    }
}
```

- [ ] **Step 2: Run and watch it fail**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*ShooterMathTest' --no-daemon
```

Expected: compile failure, `cannot find symbol ShooterMath`.

- [ ] **Step 3: Implement**

```java
package org.firstinspires.ftc.teamcode;

/** Unit conversions for the flywheel encoders. */
public final class ShooterMath {
    private ShooterMath() {}

    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond * 60.0 / ticksPerRev;
    }
}
```

- [ ] **Step 4: Run the tests**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*ShooterMathTest' --no-daemon
```

Expected: 2 tests passed.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShooterMath.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShooterMathTest.java
git commit -m "feature: shooter RPM conversions with tests"
```

---

### Task 6: Shooter subsystem, spin-up toggle and fire

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`

**Interfaces:**
- Consumes: `ShooterMath` (Task 5), `Constants.SHOOTER_*` (Task 3), `IndexerSubsystem.feed()` (Task 4).
- Produces: `ShooterSubsystem.spinUp()`, `idle()`, `toggle()`, `isRunning()`, `atSpeed()`, `getLeftRpm()`, `getRightRpm()`. Task 8 reads the RPM getters for telemetry.

- [ ] **Step 1: Write `ShooterSubsystem.java`**

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterMath;

/** Two flywheels, one motor each, held at a shared RPM by the hub's velocity controller. */
public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private boolean running;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_LEFT);
        right = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_RIGHT);
        left.setDirection(Constants.SHOOTER_LEFT_DIRECTION);
        right.setDirection(Constants.SHOOTER_RIGHT_DIRECTION);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setVelocityPIDFCoefficients(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D, Constants.SHOOTER_F);
        }
    }

    public void spinUp() {
        running = true;
        double tps = ShooterMath.rpmToTicksPerSecond(Constants.SHOOTER_SETPOINT_RPM, Constants.SHOOTER_TICKS_PER_REV);
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    public void idle() {
        running = false;
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public void toggle() {
        if (running) idle(); else spinUp();
    }

    public boolean isRunning() {
        return running;
    }

    /** True when both wheels are within tolerance of the setpoint. */
    public boolean atSpeed() {
        return running
                && Math.abs(getLeftRpm() - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM
                && Math.abs(getRightRpm() - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM;
    }

    public double getLeftRpm() {
        return ShooterMath.ticksPerSecondToRpm(left.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
    }

    public double getRightRpm() {
        return ShooterMath.ticksPerSecondToRpm(right.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
    }
}
```

- [ ] **Step 2: Wire it into `Robot.java`**

Add import `import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;`, field `public final ShooterSubsystem shooter;`, and construct `shooter = new ShooterSubsystem(hardwareMap);` after `indexer`.

In `periodic()`, before the trigger block, add:

```java
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter.toggle();
        }
        boolean fire = driver.isDown(GamepadKeys.Button.X);
```

Replace the trigger block with this version, which adds the fire branch:

```java
        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else if (fire && shooter.atSpeed()) {
            // Fire: feed only while the flywheels are at speed so every shot leaves at the same velocity.
            intake.stop();
            indexer.feed();
        } else {
            intake.stop();
            indexer.stop();
        }
```

In `stop()`, before `cancelAll()`, add `shooter.idle();`.

- [ ] **Step 3: Build**

```bash
./gradlew :TeamCode:assembleDebug --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git commit -m "feature: closed-loop shooter with spin-up toggle and fire on X"
```

---

### Task 7: Vision subsystem, alliance select and aim assist

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/VisionSubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Main.java`

**Interfaces:**
- Consumes: SDK v12 `AprilTagProcessor`, `AprilTagClusterDetection`, `AprilTagGameDatabase.getCurrentGameTagLibrary()`, `VisionPortal`; `Alliance` and `Constants.WEBCAM`, `AIM_*` (Task 3).
- Produces: `VisionSubsystem.setAlliance(Alliance)`, `getAlliance()`, `hasTarget()`, `getBearingDeg()`, `getTargetName()`, `stopLiveView()`, `close()`; `Robot.initLoop()`, `Robot.start()`.

Background: the v12 BIOBUZZ library defines four clusters named `RED SCORING` (tags 30 to 33), `RED AUDIENCE` (34 to 37), `BLUE AUDIENCE` (38 to 41) and `BLUE SCORING` (42 to 45). `AprilTagClusterDetection.metadata.name` carries that name, and `ftcPose.bearing` is the horizontal angle to the cluster origin in degrees, positive to the left. The cluster origin is the centre of the Cell opening.

- [ ] **Step 1: Write `VisionSubsystem.java`**

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * One webcam looking for our alliance's Hive cell. BIOBUZZ cells move, so this is for aiming only.
 * Cluster names in the SDK library are "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING".
 */
public class VisionSubsystem extends SubsystemBase {
    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private Alliance alliance = Alliance.RED;
    private AprilTagClusterDetection target;

    public VisionSubsystem(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.WEBCAM))
                .addProcessor(processor)
                .build();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /** Picks the best-seen cluster belonging to our alliance from the latest frame. */
    @Override
    public void periodic() {
        AprilTagClusterDetection best = null;
        for (AprilTagDetection d : processor.getDetections()) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (!c.metadata.name.startsWith(alliance.name())) continue;
            if (best == null || c.percentClusterFound > best.percentClusterFound) best = c;
        }
        target = best;
    }

    public boolean hasTarget() {
        return target != null;
    }

    /** Horizontal angle to the cell opening, degrees, positive to the left. Only valid when hasTarget(). */
    public double getBearingDeg() {
        return target.ftcPose.bearing;
    }

    public String getTargetName() {
        return target == null ? "none" : target.metadata.name;
    }

    /** Turn off the Driver Station preview once the match starts to save CPU. */
    public void stopLiveView() {
        portal.stopLiveView();
    }

    public void close() {
        portal.close();
    }
}
```

- [ ] **Step 2: Wire it into `Robot.java`**

Add imports:

```java
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
```

Add field `public final VisionSubsystem vision;` and construct `vision = new VisionSubsystem(hardwareMap);` after `shooter`.

Add these two methods after `init()`:

```java
    /** Called repeatedly while the OpMode sits in INIT. D-pad left = red, right = blue. */
    public void initLoop() {
        driver.readButtons();
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) vision.setAlliance(Alliance.RED);
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) vision.setAlliance(Alliance.BLUE);
        telemetry.addData("Alliance (dpad L/R)", vision.getAlliance());
        telemetry.addData("Camera sees", vision.getTargetName());
        telemetry.update();
    }

    /** Called once when the driver presses START. */
    public void start() {
        vision.stopLiveView();
    }
```

`initLoop()` needs the scheduler to have run `vision.periodic()` for "Camera sees" to update, so add `CommandScheduler.getInstance().run();` as its first line after `driver.readButtons()`.

Replace the drive call in `periodic()` with aim assist:

```java
        boolean fieldCentric = !driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
        double rotate = driver.getRightX();
        if (driver.isDown(GamepadKeys.Button.Y) && vision.hasTarget()) {
            rotate = aimRotation(vision.getBearingDeg());
        }
        drive.drive(driver.getLeftY(), driver.getLeftX(), rotate,
                fieldCentric, odometry.getPose().getHeading(AngleUnit.RADIANS));
```

Add this private method at the bottom of the class:

```java
    /** P-controller from tag bearing (deg, +left) to clockwise rotation power. */
    private static double aimRotation(double bearingDeg) {
        if (Math.abs(bearingDeg) < Constants.AIM_DEADBAND_DEG) return 0;
        double rotate = -Constants.AIM_KP * bearingDeg;
        return Math.max(-Constants.AIM_MAX_ROTATE, Math.min(Constants.AIM_MAX_ROTATE, rotate));
    }
```

In `stop()`, add `vision.close();` after `shooter.idle();`.

- [ ] **Step 3: Call the new hooks from `Main.java`**

Add after `init()`:

```java
    @Override
    public void init_loop() {
        robot.initLoop();
    }

    @Override
    public void start() {
        robot.start();
    }
```

- [ ] **Step 4: Build**

```bash
./gradlew :TeamCode:assembleDebug --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git commit -m "feature: AprilTag aim assist on the alliance cell, alliance picked in init"
```

---

### Task 8: Telemetry, README table and checklist

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`
- Modify: `README.md`

**Interfaces:**
- Consumes: `ShooterSubsystem.getLeftRpm()`, `getRightRpm()`, `atSpeed()`, `isRunning()`; `VisionSubsystem.hasTarget()`, `getBearingDeg()`, `getAlliance()`; `OdometrySubsystem.getPose()`.

- [ ] **Step 1: Add the voltage sensor and telemetry to `Robot.java`**

Add import `import com.qualcomm.robotcore.hardware.VoltageSensor;` and field `private final VoltageSensor battery;`. In the constructor, after the bulk-caching loop:

```java
        battery = hardwareMap.voltageSensor.iterator().next();
```

Replace the telemetry lines at the end of `periodic()` with:

```java
        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Pose", odometry.getPose());
        telemetry.addData("Shooter", "%s target %.0f  L %.0f  R %.0f  %s",
                shooter.isRunning() ? "ON" : "off", Constants.SHOOTER_SETPOINT_RPM,
                shooter.getLeftRpm(), shooter.getRightRpm(), shooter.atSpeed() ? "AT SPEED" : "");
        telemetry.addData("Tag bearing", vision.hasTarget() ? String.format("%.1f deg", vision.getBearingDeg()) : "no target");
        telemetry.addData("Battery", "%.1f V", battery.getVoltage());
        telemetry.update();
```

- [ ] **Step 2: Add the configuration table and checklist to the README**

Append to `README.md`:

```markdown
## Control Hub configuration

The Robot Controller configuration must use these names. The PR template asks
you to keep this table current when you add or move hardware.

| Config name | Type | Hub / port | Mechanism |
| --- | --- | --- | --- |
| `front_left_drive` | goBILDA 5202/3/4 series | | Drive |
| `front_right_drive` | goBILDA 5202/3/4 series | | Drive |
| `back_left_drive` | goBILDA 5202/3/4 series | | Drive |
| `back_right_drive` | goBILDA 5202/3/4 series | | Drive |
| `intake` | goBILDA 5202/3/4 series | | Intake |
| `indexer` | goBILDA 5202/3/4 series | | Indexer |
| `shooter_left` | goBILDA 5202/3/4 series | | Shooter |
| `shooter_right` | goBILDA 5202/3/4 series | | Shooter |
| `pinpoint` | goBILDA Pinpoint Odometry Computer (I2C) | | Odometry |
| `Webcam 1` | Webcam | USB | Vision |

Fill in the hub and port column when the robot is wired.

## Driver controls

One driver on gamepad 1. Pick the alliance during INIT with D-pad left (red)
or right (blue).

| Input | Action |
| --- | --- |
| Left stick | Translate, field-centric |
| Right stick X | Rotate |
| Left bumper (hold) | Robot-centric translate |
| A | Zero heading |
| Right trigger (hold) | Intake and indexer run |
| Left trigger (hold) | Intake and indexer reverse |
| Right bumper | Shooter on / off |
| X (hold) | Fire (indexer feeds once the shooter is at speed) |
| Y (hold) | Aim at our Hive cell |

## Testing on the robot

Run through these after any change to the matching subsystem and record the
result in the PR.

1. Drive: each motor spins forward on positive power, the robot drives
   straight, strafes right on stick right, and field-centric holds direction
   after a spin.
2. Odometry: Pinpoint LED green. X grows driving forward, Y grows driving left,
   heading grows turning anticlockwise. Spinning in place moves X/Y under
   100 mm. Returning to the start reads under 10 mm.
3. Intake and indexer: run, reverse, stop. The indexer holds a ball when
   stopped.
4. Shooter: reaches the setpoint, telemetry RPM within tolerance, the AT SPEED
   flag appears, and a Pollen lands in the Cell from the practice spot.
5. Vision: telemetry shows a bearing with a Cell tag in view, and holding Y
   turns the robot toward it and settles.

Tuning values (shooter RPM and PIDF, aim gain, Pinpoint pod offsets) live in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`.
```

- [ ] **Step 3: Build and test**

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java README.md
git commit -m "docs: driver telemetry, config table, controls and on-robot checklist"
```

---

## After the plan

Everything above compiles and passes the JVM tests without a robot. The on-robot checklist in the README is the acceptance test. Expect to change `SHOOTER_SETPOINT_RPM`, the `SHOOTER_*` PIDF values, `AIM_KP` and the two `PINPOINT_*_OFFSET_MM` values on the first day with the robot; each is one line in `Constants.java`.
