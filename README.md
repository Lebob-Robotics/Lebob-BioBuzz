# Lebob-BioBuzz

Robot code for Lebob Robotics, team 29550, competing in FIRST Tech Challenge
BIOBUZZ 2026/27 at the Western Australia Qualifier.

This repository is an unmodified import of the FIRST Tech Challenge SDK with
our own code in `TeamCode`. 

## Requirements

- **JDK 17.** <https://adoptium.net/temurin/releases/?version=17>
- **Android SDK packages** <https://developer.android.com/studio#command-line-tools-only> (listed below)
- **FIRST Tech Challenge SDK v11.2.1** <https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v11.2.1> (already in this repository)

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
