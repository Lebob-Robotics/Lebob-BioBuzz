# Analysis scripts

Python scripts for PsiKit logs, the tuning CSVs and the field webcam, copied from `sotm-tools` (the folder beside
this repo that also holds the 2.4 GB of logs and video, which stay there). Run them from this folder or by path;
each finds its own imports.

Setup: `python3 -m venv .venv && .venv/bin/pip install numpy opencv-python pillow websockets`. Only the webcam
scripts need OpenCV and Pillow, and only the Panels scripts need websockets. Scripts that pull from the Hub use
`adb` at `192.168.43.1:5555`. Pulled logs and outputs go to `logs/`, `robotcam/` and the like here, which git
ignores.

## Logs (PsiKit `.rlog`)
- `rlogparse.py`: the log reader the others use; `parse(path)` gives `{key: [(t, value)]}`.
- `quick.py LOG`: field test phase, pose range and vision correction reasons at a glance.
- `lap_an.py LOG`: each trial of the SOTM field test: how soon and how often the shot was READY, and which gate
  (RPM, hood, heading, aim) held it back.
- `vision_test.py`, `vcal_status.py`, `vcal_peek.py LOG`: camera status, frame counts and tag ids per camera.
- `exposure_sweep.py LOG`: for each exposure and gain in a camera sweep, how often each tag is seen, its decision
  margin, range and jitter.
- `battwatch.py`: follows the newest log on the Hub and keeps its battery samples.
- `pedro_sysid.py OUT.json [LOG_GLOB]`: full-power and stopping runs for Pedro's drive constants (default glob
  `logs/*20261113*.rlog`, the 13 Nov logs `pedro/Constants.java` was fitted to).

## Tuning CSVs
- `tuning_report.py [--no-pull] [--session S] [--balls]`: pulls Shot Tuning's `ShotTuning_v2.csv` and summarises
  the balls into table rows.
- `tuning_to_fit.py [--mode HOODED|BACKWARD] [out.csv]`: Shot Tuning's standing balls as the grid CSV that
  `tools/shots/fit-shots.js` fits.
- `sotm_report.py [SotmTuning.csv] [--pull] [--session S]`: how SOTM Tuning's moving balls did, and fitted
  corrections for `SotmConstants`. Reads the shot table from this repo's `ShotTable.java`.
- `gridreport.py LOG [BALLS]`: per-ball rows and webcam frames for a grid run, for `fit-shots.js`.

## Field webcam
These need the webcam recorder under `/var/mnt/timelapse`, and `camera.json` and `reference.jpg` here (copy them from
`sotm-tools`, or run `camera.py calibrate`). Scripts that take a log need `LOG.clock` beside it, the PC clock minus
the Hub's uptime for that boot; `autodual.py` writes it when it pulls a log, or pass `--offset`.
- `replay.py START DUR OUT`: cuts the recording at a wall-clock time; `vidframes.py` gets exact frames.
- `camera.py locate|calibrate|scan|crop`: finds the Hive in the webcam picture, so crops survive the camera moving.
- `balltrack.py`, `ballreport.py LOG`, `trackshots.py LOG`: ball flights from the webcam, matched to the log's feeds.
- `syncchk.py LOG`: checks a log's clock against the webcam.
- `dual.py LOG`, `autodual.py`: side-by-side videos of a run (webcam, robot camera, field map, shot state).

## Panels
- `panels.py`: websocket helpers for scripted runs; `panels_run.py NAME SECONDS` and `panels_stop.py` start and
  stop an OpMode through Panels.
