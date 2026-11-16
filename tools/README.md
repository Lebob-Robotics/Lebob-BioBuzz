# Tools

Laptop-side tools for the robot code. Nothing here runs on the robot.

| Path | What it is |
| --- | --- |
| `verify.sh` | Every check a refactor commit must pass: the debug APK builds, all unit tests pass (including the constants snapshot), and no log key changed |
| `check_log_keys.py`, `log-keys.txt` | Lists the PsiKit log keys the code writes and compares them with `log-keys.txt`; `--write` rewrites it |
| `deploy.sh` | Builds TeamCode and pushes it to the Control Hub with Sloth, failing within seconds when the Hub can't be reached; `--force` pushes while an OpMode runs |
| `start_robot.sh` | Brings up the robot Wi-Fi, adb, Panels and the controller bridge on this PC |
| `panels_gamepad_bridge.py` | Drives the robot from a controller on this PC through Panels |
| `shots/` | Shot physics, the hood and RPM table generator (`ShotTable.java`) and fits to measured shots. See its README |
| `flywheel/` | Sizes the flywheel mass so four balls in a row stay within the RPM tolerance. See its README |
| `design.js` | The mk2 shooter design pass: chains the hood sweep and the flywheel sizing |
| `camera-calibration/` | ChArUco lens calibration for a webcam; `results/` holds CameraL and CameraR at 1280x720 |
| `vision/` | Camera exposure report from a match log. See its README |
| `solo/` | Plays the solo auto's branches (tipped, not tipped, and Triple's partner preloads) together on the field, from `SoloAutoSimTest`. See its README |
| `analysis/` | Python scripts for PsiKit logs, the tuning CSVs and the field webcam. See its README |

The measurements behind the tables are in `../data/`.
