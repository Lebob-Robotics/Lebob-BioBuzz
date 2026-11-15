# Lebob-BioBuzz

Robot code for Lebob Robotics, team 29550, competing in FIRST Tech Challenge
BIOBUZZ 2026/27 at the Western Australia Qualifier.

This repository is an unmodified import of the FIRST Tech Challenge SDK with
our own code in `TeamCode`.

## OpModes

| Driver Station name | Group | Class | What it does |
| --- | --- | --- | --- |
| Lebot Teleop | Robot | `opmodes/Main` | The match TeleOp. LT loads, RB fires a standing volley, RT shoots on the move. Reads Volley Tuning's CSV for its volley settings |
| Lebot Auto Red, Lebot Auto Blue | Robot | `auto/ScoringAutoRed`, `ScoringAutoBlue` | The scoring auto: preloads into the Cell that is up, the Garden into the other, our wall's Flower, then park |
| Lebot Corner Red, Lebot Corner Blue | Robot | `auto/CornerAutoRed`, `CornerAutoBlue` | The corner auto: preloads from the rear corner, a wait there, our wall's Flower, a volley into the AUDIENCE Cell, then park, shooting what's left |
| Shoot Auto | Robot | `auto/ShootAuto` | Drives forward, fires the load as one volley, parks in the loading zone. X blue, B red in INIT |
| Volley Tuning | Tuning | `opmodes/tuning/VolleyTuning` | Four-ball volleys at Shot Tuning's spots; writes `VolleyTuning_v3.csv` |
| Shot Tuning | Tuning | `opmodes/tuning/ShotTuning` | Standing shots at test spots, to find each spot's scoring hood (or RPM) range and aim trim; writes `ShotTuning_v2.csv` |
| SOTM Tuning | Tuning | `opmodes/tuning/SotmTuning` | Planned shoot-on-the-move passes, one ball each; writes `SotmTuning.csv` |
| Special Shot Calibration | Tuning | `opmodes/tuning/SpecialShotCalibration` | The lob over our Hive into its far Cell; writes `SpecialShot.csv` |
| Vision Calibration | Tuning | `opmodes/check/VisionCalibration` | Measures tags, and in INIT sweeps the camera exposure. Nothing moves |
| Odometry Calibration | Tuning | `opmodes/check/OdometryCalibration` | Measures the Pinpoint pod offsets by spinning in place, for `hardware/PinpointSettings` |
| Indexer Intake Test | Test | `opmodes/check/IndexerIntakeTest` | Runs each indexer servo and the intake alone and shows what each draws |
| Ramp Servo Check | Test | `opmodes/check/RampServoCheck` | Finds the Pollen ramp's stowed and deployed servo pulses |

Each class's comment lists its controls. The autos preselect Lebot Teleop.

## Robot configuration

The names in the Control Hub's robot configuration, from `hardware/HardwareNames.java`. They must match exactly.
`HardwareNamesReadmeTest` fails when this table and `HardwareNames` disagree.

| Name | Device | Port | Notes |
| --- | --- | --- | --- |
| `DriveFL` | Motor | | Front left drive |
| `DriveFR` | Motor | | Front right drive |
| `DriveBL` | Motor | | Back left drive |
| `DriveBR` | Motor | | Back right drive |
| `ShooterL` | Motor | | Left flywheel |
| `ShooterR` | Motor | | Right flywheel |
| `IntakeMotor` | Motor | | Intake. Its encoder port reads the hood's REV encoder |
| `HoodMotor` | Motor | | Hood |
| `IndexerServoL` | Continuous rotation servo | Expansion Hub servo 0 | Upstream indexer rollers |
| `IndexerServoR` | Continuous rotation servo | Control Hub servo 0 | Last indexer roller |
| `FeederServo` | Continuous rotation servo | Control Hub servo 1 | Feeder |
| `FeederServoR` | Continuous rotation servo | | Optional paired feeder; used ahead of `FeederServo` when present |
| `FeederServoL` | Continuous rotation servo | | Optional paired feeder |
| `RampServo` | Servo | Expansion Hub servo 1 | Pollen ramp. Optional: without it the telemetry says so |
| `pinpoint` | goBILDA Pinpoint | I2C | Odometry |
| `CameraL` | Webcam | USB | Left camera |
| `CameraR` | Webcam | USB | Right camera |
| `BeamBreakT` | Digital device | | Not needed while `IntakeSubsystem.BEAM_BREAKS_WIRED` is false |
| `BeamBreakB` | Digital device | | Not needed while `IntakeSubsystem.BEAM_BREAKS_WIRED` is false |

## Requirements

- **JDK 17.** <https://adoptium.net/temurin/releases/?version=17>
- **Android SDK packages** <https://developer.android.com/studio#command-line-tools-only> (listed below)
- **FIRST Tech Challenge SDK v12.0** <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0> (already in this repository, the BIOBUZZ season release)

The required SDK packages:

| Package                | Why                                               |
| ---------------------- | ------------------------------------------------- |
| `platforms;android-30` | `build.common.gradle` sets `compileSdkVersion 30` |
| `build-tools;35.0.0`   | the revision AGP 8.13.2 resolves to               |
| `platform-tools`       | provides `adb`, used to deploy to the Control Hub |

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
./gradlew :TeamCode:assembleDebug && adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk && adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
```

### Using Android Studio instead

Android Studio download:

<https://developer.android.com/studio>

Open the project, let it sync, connect to the Control Hub's wifi network, and
run the `TeamCode` configuration. Android Studio does the same `adb install` and
launch shown above. If the Hub does not appear in the device dropdown, run
`adb connect 192.168.43.1:5555` in a terminal first; Android Studio picks up
devices from the same adb server.

## Deploy with Sloth (hot reload)

The full install above takes a long time because it sends the whole APK. For
day-to-day changes, deploy with [Sloth](https://github.com/Dairy-Foundation/Sloth),
which sends only the TeamCode classes and reloads them on the Hub in about a
second. The Gradle build already has Sloth wired in (`TeamCode/build.gradle`
applies the `dev.frozenmilk.sinister.sloth.load` plugin and depends on
`dev.frozenmilk.sinister:Sloth`), so no extra setup is needed beyond the
requirements above.

```
./gradlew :TeamCode:deploySloth
```

`deploySloth` connects to the Hub at `192.168.43.1` with adb by itself and
pushes the change. Run it again after every edit. Changes are kept across Hub
restarts. Only classes in `org.firstinspires.ftc.teamcode` are reloaded, so a
change elsewhere, or a change to a library, needs a full install.

From a terminal, `tools/deploy.sh` runs the same deploy with checks first. It
stops within 2 seconds when the Hub isn't reachable, where `deploySloth` alone
waits about 2 minutes. It also reconnects adb after a Hub reboot, and refuses to
deploy while an OpMode is writing its log (`tools/deploy.sh --force` deploys
anyway).

Things to know:

- **A full install removes the Sloth copy.** `installDebug` and
  `installRelease` run `removeSlothRemote` first, so the APK you install is the
  one that runs. You don't need to clear anything by hand.
- **If a Sloth deploy reports a lock file,** a previous load was probably
  interrupted. Restart the Robot Controller app or the Hub, then deploy again.
- **To remove Sloth code from the Hub without installing**, run
  `./gradlew :TeamCode:removeSlothRemote`.
- **In Android Studio,** add a Gradle run configuration for the `TeamCode`
  module with the task `:TeamCode:deploySloth`.

## Tests

```
./gradlew :TeamCode:testDebugUnitTest
tools/verify.sh
```

The unit tests run on the laptop, without the robot. `tools/verify.sh` builds the debug APK, runs every unit test
and checks the log keys; run it before every commit. These tests guard the robot's behaviour:

- `ConstantsSnapshotTest`: every static constant keeps its value, from
  `TeamCode/src/test/resources/constants/snapshot.txt`. When a constant or class moves, add the move to
  `moves.txt` beside it.
- `RobotLoopTraceTest` and `TuningOpModeTraceTest`: run the robot and the tuning OpModes in a simulation and compare
  every loop with `robot/loop-trace.txt` and `robot/tuning-trace.txt` in the same folder. A change that is meant to
  alter behaviour rewrites them with `WRITE_ROBOT_TRACE=1`; check the diff before committing it.
- `tools/check_log_keys.py`: the log keys still match `tools/log-keys.txt`. `--write` updates the list after a
  deliberate change.

## Tuning

The tuning OpModes write CSV files to `/sdcard/FIRST/` on the Control Hub. They append, so earlier sessions stay in
the file. In INIT, Y in Shot, SOTM and Volley Tuning chooses between carrying on the saved searches and starting new
ones.

1. **Shot Tuning** finds each spot's standing shot. `tools/analysis/tuning_report.py` pulls `ShotTuning_v2.csv` and
   turns it into table rows; `tools/analysis/tuning_to_fit.py` gives `tools/shots/fit-shots.js` its input. See
   `tools/shots/README.md` for regenerating the shot table.
2. **SOTM Tuning** starts each moving spot from Shot Tuning's standing aim trims (it reads `ShotTuning_v2.csv`).
   `tools/analysis/sotm_report.py` fits `SotmConstants`' moving corrections from `SotmTuning.csv`.
3. **Volley Tuning** tunes four-ball volleys at Shot Tuning's spots and writes `VolleyTuning_v3.csv`. Lebot Teleop
   and the Lebot Auto and Lebot Corner autos read this file in INIT for their volley settings, from its confirmed
   spots only (`VolleyConstants.VOLLEY_USE_TUNING_CSV`). **The file must be on the Hub.** Without it they quietly
   use the `VolleyConstants` volley instead. The regionals file is in `data/volley/`. To put it on a new or reset
   Hub (this replaces the Hub's file, so only do it when the Hub has no newer one):

   ```
   adb push data/volley/VolleyTuning_v3_2026-11-15.csv /sdcard/FIRST/VolleyTuning_v3.csv
   ```

   After tuning, keep a copy with `adb pull /sdcard/FIRST/VolleyTuning_v3.csv`.
4. **Special Shot Calibration** writes `SpecialShot.csv`, for the lob into the far Cell.

Vision Calibration (with `tools/vision/`) and Odometry Calibration are for when a camera or an odometry pod moves.

## Logs and analysis

PsiKit logs every OpMode run to `/sdcard/FIRST/PsiKit/*.rlog` on the Hub. Pull them into the analysis folder, where
git ignores them:

```
adb pull /sdcard/FIRST/PsiKit/ tools/analysis/logs/
```

Open them in AdvantageScope, or read them with the scripts in `tools/analysis/` (see its README; `quick.py` gives a
run at a glance). `Main.LIVE_LOGGING` serves the log live to AdvantageScope on port 5810; it must be false at
competition. When the robot loop hangs, `LoopWatchdog` writes the loop thread's stack to
`/sdcard/FIRST/loop_stalls.txt`, which survives the app restart that follows. `tools/README.md` lists the other
laptop tools.
