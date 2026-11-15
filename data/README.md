# Data

Measurements that TeamCode's constants and shot tables were fitted to. The robot never reads these copies: the
Hub keeps its own under `/sdcard/FIRST/`.

## calibration/2026-11-07/

The tuning OpModes' CSVs as pulled from the Hub on 7 Nov. Session ids read `19700101_...` because the Hub's clock
resets each boot.

| File | Written by | Read by |
|---|---|---|
| `ShotTuning.csv` | Shot Tuning, first format | nothing now; kept for the history |
| `ShotTuning_v2.csv` | Shot Tuning (`/sdcard/FIRST/ShotTuning_v2.csv`) | `tools/analysis/tuning_report.py`, `tuning_to_fit.py`; SOTM Tuning reads the Hub's copy for its standing shots |
| `SotmTuning.csv` | SOTM Tuning (`/sdcard/FIRST/SotmTuning.csv`) | `tools/analysis/sotm_report.py` |
| `SpecialShot.csv` | Special Shot Calibration | nothing: a header and one row from a test run |

## volley/

`VolleyTuning_v3_2026-11-15.csv` is Volley Tuning's CSV as it stood at regionals on 15 Nov. Lebot Teleop and the
autos shoot the spots it confirms, so the Hub must have it as `/sdcard/FIRST/VolleyTuning_v3.csv`:

```
adb push data/volley/VolleyTuning_v3_2026-11-15.csv /sdcard/FIRST/VolleyTuning_v3.csv
```

The unit tests read the same file as `TeamCode/src/test/resources/volley/VolleyTuning_20261115.csv`.
