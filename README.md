# Lebob-BioBuzz

Robot code for Lebob Robotics, team 29550, competing in FIRST Tech Challenge
BIOBUZZ 2026/27 at the Western Australia Qualifier.

This repository is an unmodified import of the FIRST Tech Challenge SDK with the
team's own code in `TeamCode`. Everything outside `TeamCode` belongs to the SDK
and is replaced wholesale when a new SDK version is merged, so team code must
not depend on edits made there.

## Requirements

Android Studio is not required. The build needs a JDK and some Android SDK
packages; Android Studio is only one way of obtaining them. The CI build in
`.github/workflows/build.yml` compiles this project with no Android Studio
installed anywhere, so the command-line path below is the one that is actually
tested on every pull request.

- **JDK 17.** This is the version CI builds with, so it is the one known to
  work. Gradle 9.1.0 and AGP 8.13.2 both predate current JDK releases, and a
  much newer JDK is not guaranteed to be supported — if the build fails with a
  class-file or unsupported-version error, check `java -version` first. Do not
  assume your system default is fine.
- **Android SDK packages**, listed below. Only three are needed.
- **FIRST Tech Challenge SDK v11.2.1**, already in this repository. Nothing
  needs to be downloaded separately.

The required SDK packages, and why each one:

| Package | Why |
| --- | --- |
| `platforms;android-30` | `build.common.gradle` sets `compileSdkVersion 30` |
| `build-tools;35.0.0` | the revision AGP 8.13.2 resolves to |
| `platform-tools` | provides `adb`, used to deploy to the Control Hub |

The NDK is not needed. `build.common.gradle` declares
`ndkVersion 21.3.6528147`, but neither module has native sources, so it is never
resolved. Do not spend the download on it.

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
uninstalled first, which is why that one keystore is deliberately still tracked
in git while `*.jks` and `*.keystore` are otherwise ignored.

## Deploy to the Control Hub

The Control Hub is an Android device and `adb` talks to it directly, so
deploying does not need an IDE either.

Connect your laptop to the wifi network the Control Hub broadcasts. The default
name starts with `FTC-` and the default password is `password`; both should have
been changed already, so ask if you do not know them. On its own network the
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

Open the project, let it sync, connect to the Control Hub's wifi network, and
run the `TeamCode` configuration. Android Studio does the same `adb install` and
launch shown above. If the Hub does not appear in the device dropdown, run
`adb connect 192.168.43.1:5555` in a terminal first; Android Studio picks up
devices from the same adb server.

## What is and is not on the deploy path

GitHub is history and review only. Nothing in this repository is on the deploy
path: pushing a branch, merging a pull request and passing the build check do
not change what is running on the robot. The only thing that changes the robot
is somebody installing a build onto the Control Hub from Android Studio.

## Control Hub configuration

The Control Hub configuration is created on the Driver Station, not in this
repository. This table is the record of what that configuration is meant to be.

| Device | Config name | Port | Notes |
| --- | --- | --- | --- |
| TODO | TODO | TODO | TODO |
| TODO | TODO | TODO | TODO |
| TODO | TODO | TODO | TODO |
| TODO | TODO | TODO | TODO |

Config names used in code must match this table exactly, including case. A name
that differs from the Control Hub configuration fails at run time, when an
OpMode is initialised, not at build time.

## Branch policy

`main` is protected:

- Changes reach `main` through a pull request, merged as a squash.
- History on `main` is linear.
- No direct pushes to `main`.
- No force pushes to `main`.

Work on a branch named `feature/<short-name>`.

## Updating the SDK

The SDK is updated by merging an upstream release tag. Add the upstream remote
once:

```
git remote add upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
```

Then, for each update, fetch the tags and merge the one you want:

```
git fetch upstream --tags
git merge <tag>
```

The BIOBUZZ SDK release is expected on 2026-09-12. Do the merge on a branch and
land it through a pull request like any other change, so the build check runs
against it before it reaches `main`.

Conflicts in the merge are a signal that something outside `TeamCode` was
edited. Keeping those files untouched is what makes SDK updates cheap.
