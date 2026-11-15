# Vision exposure from a match log

The cameras run at the `FIELD_LIGHTING` preset in `VisionSubsystem` (COMP: `COMP_EXPOSURE_MS`, `COMP_GAIN`). Every
OpMode that runs the Robot logs, per camera, everything needed to judge that setting afterwards:

| Log key (`RealOutputs/Vision/{Left,Right}/…`) | What |
| --- | --- |
| `Lighting`, `ExposureMs`, `Gain` | preset in use, and the exposure and gain the camera reported back |
| `Brightness/Mean`, `ClippedPct`, `DarkPct` | mean grey level 0-255, % pixels clipped white (≥250), % crushed black (≤20), 5 times a second (`FrameBrightness`) |
| `FramesTotal` | frames the camera delivered, for the share of frames that found a tag |
| `Vision/{Left,Right}/DecisionMargin`, `XIn`… | per-tag contrast and position (already logged) |

## After the practice match

On the robot's Wi-Fi:

```
tools/vision/pull_and_report.sh Main_log         # TeleOp
tools/vision/pull_and_report.sh ScoringAuto      # the auto
tools/vision/exposure_report.py tools/vision/logs/*.rlog   # again, or several at once
```

The report gives, per camera: fps, brightness, how many frames saw a tag, and decision margins by distance and by
robot speed. It ends with a suggestion. Margin is what counts (home was ~140 at 1-1.5 m; the pose correction drops a
tag under `MIN_DECISION_MARGIN`, 20). Brightness says which way to move:

- **Too bright** (mean over ~150, or several % clipped): lower `COMP_GAIN` first, then `COMP_EXPOSURE_MS`.
- **Too dark** (mean under ~60, a lot crushed black) and margins low: raise `COMP_GAIN` first; longer exposure blurs
  tags while driving (compare the "moving" and "still" margins).

Both cameras share the COMP values, so go with the weaker camera. Change the two numbers in `VisionSubsystem`,
`./gradlew :TeamCode:deploySloth`, and check the next match's report. Panels is off at events (R704), so the values can't
be changed live.

`test_exposure_report.py` checks the report on a synthetic log.
