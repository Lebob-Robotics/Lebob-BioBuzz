#!/usr/bin/env python3
"""Shot Tuning's standing hooded balls (logs/ShotTuning_v2.csv), each with its last mark, as the grid CSV that
tools/shots/fit-shots.js reads: hood is the angle measured at the feed, rpm the wheels' mean then.
Marks: IN in, BOUNCE out, SHORT short, LONG long; left or right alone say nothing about range and are left out.
    python3 tuning_to_fit.py [--mode HOODED|BACKWARD] [out.csv]"""
import csv
import os
import sys
from collections import OrderedDict

mode = "HOODED"
args = sys.argv[1:]
if args[:1] == ["--mode"]:
    mode, args = args[1], args[2:]
out = args[0] if args else f"tuning_{mode.lower()}_fit.csv"
shots = OrderedDict()
for r in csv.DictReader(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "ShotTuning_v2.csv"))):
    key = (r["session"], r["shot"])
    if r["kind"] == "SHOT":
        shots[key] = {"row": r, "mark": None}
    elif key in shots:
        shots[key]["mark"] = r["kind"]
n = 0
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["log", "ball", "spot", "lip_m", "hood", "hood_tgt", "rpm", "rpm_tgt", "result"])
    for (session, shot), s in shots.items():
        r, mark = s["row"], s["mark"]
        if r["mode"] != mode or r["pass"] or not mark:
            continue
        parts = mark.split("+")
        result = ("in" if mark == "IN" else "out" if mark == "BOUNCE" else "short" if "SHORT" in parts
                  else "long" if "LONG" in parts else None)
        if result is None:
            continue
        w.writerow([session, shot, r["spot"], r["lip_m"], r["hood_at_feed_deg"], r["launch_deg"], r["rpm_at_feed"],
                    r["target_rpm"], result])
        n += 1
print(f"{n} {mode} balls -> {out}")
