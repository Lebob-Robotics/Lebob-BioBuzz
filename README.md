# Lebob-BioBuzz

Robot code for Lebob Robotics, team 29550, competing in FIRST Tech Challenge
BIOBUZZ 2026/27 at the Western Australia Qualifier.

This repository is an unmodified import of the FIRST Tech Challenge SDK with
our own code in `TeamCode`. 

## Requirements

- **JDK 17.** <https://adoptium.net/temurin/releases/?version=17>
- **Android SDK packages** <https://developer.android.com/studio#command-line-tools-only> (listed below)
- **FIRST Tech Challenge SDK v12.0** <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0> (already in this repository, the BIOBUZZ season release)

The required SDK packages:

| Package | Why |
| --- | --- |
| `platforms;android-30` | `build.common.gradle` sets `compileSdkVersion 30` |
| `build-tools;35.0.0` | the revision AGP 8.13.2 resolves to |
| `platform-tools` | provides `adb`, used to deploy to the Control Hub |

### Getting the SDK without Android Studio

Download the "command line tools only" package from
<https://developer.android.com/studio#command-line-tools-only> and unpack it so
that `sdkmanager` ends up at `$ANDROID_HOME/cmdline-tools/latest/bin`:

```
export ANDROID_HOME="$HOME/Android/Sdk"
mkdir -p "$ANDROID_HOME/cmdline-tools"
unzip commandlinetools-linux-*.zip -d "$ANDROID_HOME/cmdline-tools"
mv "$ANDROID_HOME/cmdline-tools/cmdline-tools" "$ANDROID_HOME/cmdline-tools/latest"
export PATH="$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/platform-tools:$PATH"
```

Then accept the licences and install the three packages:

```
sdkmanager --licenses
sdkmanager "platform-tools" "platforms;android-30" "build-tools;35.0.0"
```

Put `ANDROID_HOME` and the `PATH` line in your shell profile so they survive a
new terminal. Gradle finds the SDK through `ANDROID_HOME`; if you would rather
not use an environment variable, create a `local.properties` file in the
repository root containing `sdk.dir=/home/you/Android/Sdk` instead. That file is
gitignored and must not be committed, because the path is specific to your
machine.

If you already have Android Studio, you already have all of this. Point
`ANDROID_HOME` at the SDK it installed, usually `~/Android/Sdk`.

## Build

From the repository root:

```
./gradlew :TeamCode:assembleDebug
```

The APK is written to
`TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.

Every build from this repository is signed with `libs/ftc.debug.keystore`, the
debug key shipped with the SDK. Keeping that key identical for everyone on the
team is what lets a new build replace an older one on the Hub in place. A build
signed with a different key is rejected with a signature mismatch and has to be
uninstalled first.

## Deploy to the Control Hub

The Control Hub is an Android device and `adb` talks to it directly, so
deploying does not need an IDE either.

Connect your laptop to the wifi network the Control Hub broadcasts. On its own network the
Control Hub is always at `192.168.43.1`, with its web interface on port 8080 and
adb listening on port 5555.

```
adb connect 192.168.43.1:5555
adb devices
```

`adb devices` should list `192.168.43.1:5555` as `device`. If it says
`unauthorized` or nothing appears, run `adb kill-server` and connect again.
Then install the APK and start the Robot Controller:

```
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
```

Installing stops the app that was running, and the Hub only autostarts the
Robot Controller at boot, so it will not come back on its own. The `am start`
above restarts it; `adb reboot` also works and takes longer.

A one-line version of build-and-deploy, once the connection is up:

```
./gradlew :TeamCode:assembleDebug && \
  adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk && \
  adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
```

### Using Android Studio instead

Android Studio download: 

<https://developer.android.com/studio>

Open the project, let it sync, connect to the Control Hub's wifi network, and
run the `TeamCode` configuration. Android Studio does the same `adb install` and
launch shown above. If the Hub does not appear in the device dropdown, run
`adb connect 192.168.43.1:5555` in a terminal first; Android Studio picks up
devices from the same adb server.

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
| X (hold) | Fire: feeds only when at speed, and when aiming also only with a valid shot and heading inside the Cell width |
| Y (hold) | Aim: rotation tracks the solved shot heading. The shooter target follows the table whenever the Cell position is known, aiming or not. |
| D-pad up / down | Trim every shooter target by ±50 RPM for the rest of the run |

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

### Shoot on the move

Tools and the measurement procedure are in `tools/shots/README.md`. Verify in
this order, recording hits in the PR:

0. Velocity frame: strafe left with the robot facing +X and confirm the Pose
   telemetry shows `v` growing in the second component, then turn 90° and
   repeat; the same component must still grow. If it swaps, the Pinpoint
   reports robot-frame velocity and `OdometrySubsystem.getVelocity()` must
   rotate it by the heading before the solver sees it.
1. Stationary at 1.0, 1.5, 2.0 and 2.5 m: at least 8 of 10 Pollen in. Fix the
   table inputs before moving on.
2. Strafing across the Cell at a steady speed: 7 of 10.
3. Driving toward and away at a steady speed: 7 of 10, and "NO SHOT: CLOSING
   TOO FAST" appears near the speed the band chart predicts.
4. Free driving: log, video, count, fix what the log shows.
