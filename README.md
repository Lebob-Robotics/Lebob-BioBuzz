# Lebob-BioBuzz

Robot code for Lebob Robotics, team 29550, competing in FIRST Tech Challenge
BIOBUZZ 2026/27 at the Western Australia Qualifier.

This repository is an unmodified import of the FIRST Tech Challenge SDK with the
team's own code in `TeamCode`. Everything outside `TeamCode` belongs to the SDK
and is replaced wholesale when a new SDK version is merged, so team code must
not depend on edits made there.

## Requirements

- Android Studio, with the Android SDK it installs.
- JDK 17. The Android Gradle Plugin used by this SDK version will not run on an
  older JDK, and Android Studio's bundled JDK is the simplest way to get it.
- FIRST Tech Challenge SDK v11.2.1. This is already in the repository; nothing
  needs to be downloaded separately.

## Build and deploy

Build the team's APK from the repository root:

```
./gradlew :TeamCode:assembleDebug
```

To put that build on the robot, open the project in Android Studio, connect to
the Control Hub over ADB — either by USB or over wifi to the Control Hub's
network — and run the `TeamCode` configuration. Android Studio installs the APK
onto the Control Hub directly.

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
