#!/usr/bin/env python3
"""Pull /sdcard/FIRST/ShotTuning_v2.csv from the hub and summarise the Shot Tuning balls.

  python3 tuning_report.py [--session 19700101_080306] [--no-pull] [--balls]

Each ball has a SHOT row as it leaves. A mark adds a row with the same shot number: IN, BOUNCE (in, then out), or
the miss's directions joined by + (SHORT, LONG, LEFT, RIGHT, e.g. SHORT+LEFT; MISS with none given). A ball's last
mark counts; unmarked balls are counted apart.

Prints, per spot and mode, the standing balls by setting and the scoring range the search found; the moving passes
by direction; and a confounder check: whether the heading error, the RPM at the feed, the hood at the feed and the
battery differ between balls that went in and balls that missed one way, with the same setting.
"""
import argparse
import csv
import os
import subprocess
from collections import defaultdict

HUB = "192.168.43.1:5555"
REMOTE = "/sdcard/FIRST/ShotTuning_v2.csv"
LOCAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "ShotTuning_v2.csv")
OUTCOMES = ["IN", "BOUNCE", "SHORT", "LONG", "LEFT", "RIGHT", "MISS", "unmarked"]


def f(r, key, default=float("nan")):
    try:
        return float(r.get(key) or default)
    except ValueError:
        return default


def outcomes(mark):
    """The outcome classes one mark counts in: a SHORT+LEFT miss is both short and left."""
    if mark is None:
        return ["unmarked"]
    if mark in ("IN", "BOUNCE", "MISS"):
        return [mark]
    return mark.split("+")


def tally(balls, marks):
    t = defaultdict(int)
    for key, _ in balls:
        for o in outcomes(marks.get(key)):
            t[o] += 1
    return t


def tally_text(t, fired):
    parts = [f"{t['IN']}/{fired} in"]
    for o in OUTCOMES[1:]:
        if t[o]:
            parts.append(f"{t[o]} {o.lower()}")
    return ", ".join(parts)


def mean(xs):
    xs = [x for x in xs if x == x]
    return sum(xs) / len(xs) if xs else float("nan")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--session", help="only this session (prefix match)")
    p.add_argument("--no-pull", action="store_true", help="use the copy already in logs/")
    p.add_argument("--balls", action="store_true", help="also list every ball")
    a = p.parse_args()
    if not a.no_pull:
        os.makedirs(os.path.dirname(LOCAL), exist_ok=True)
        subprocess.run(["adb", "-s", HUB, "pull", REMOTE, LOCAL], check=True, stdout=subprocess.DEVNULL)
    shots, marks = {}, {}
    with open(LOCAL, newline="") as fh:
        for r in csv.DictReader(fh):
            if a.session and not r["session"].startswith(a.session):
                continue
            key = (r["session"], int(r["shot"]))
            if r["kind"] == "SHOT":
                shots[key] = r
            else:
                marks[key] = r["kind"]
    if not shots:
        print("no shots")
        return
    standing = {k: r for k, r in shots.items() if not r.get("pass")}
    moving = {k: r for k, r in shots.items() if r.get("pass")}
    print(f"{len(shots)} balls: {len(standing)} standing, {len(moving)} on moving passes; "
          f"{sum(1 for k in shots if k not in marks)} unmarked")

    if a.balls:
        print(f"\n{'session':16} {'#':>3} {'mode':8} {'spot':4} {'lip':>5} {'rpm':>5} {'@feed':>5} {'launch':>6} "
              f"{'@feed':>6} {'head':>5} {'trim':>5} {'V':>5} {'tow':>5} {'side':>5} {'pass':12} {'search':14} result")
        for key in sorted(shots):
            r = shots[key]
            print(f"{key[0]:16} {key[1]:3} {r['mode']:8} {r.get('spot', ''):4} {f(r, 'lip_m'):5.2f} "
                  f"{f(r, 'target_rpm'):5.0f} {f(r, 'rpm_at_feed'):5.0f} {f(r, 'launch_deg'):6.1f} "
                  f"{f(r, 'hood_at_feed_deg'):6.1f} {f(r, 'head_err_deg'):+5.1f} {f(r, 'head_trim_deg', 0):+5.1f} "
                  f"{f(r, 'battery_v'):5.2f} {f(r, 'toward_mps'):+5.2f} {f(r, 'sideways_mps'):+5.2f} "
                  f"{r.get('pass', ''):12} {r.get('search', ''):14} {marks.get(key, '-')}")

    # Standing balls per mode and spot (a ball without a spot goes by its distance), then by setting
    print("\nSTANDING, per spot: rpm / launch deg [aim trim, left +] -> outcome")
    spots = defaultdict(list)
    for key, r in standing.items():
        where = r.get("spot") or f"{round(f(r, 'lip_m') / 0.15) * 0.15:.2f}m"
        spots[(r["mode"], where)].append((key, r))
    for (mode, where) in sorted(spots):
        balls = spots[(mode, where)]
        lip = mean(f(r, "lip_m") for _, r in balls)
        print(f"\n  {mode} {where} (lip {lip:.2f} m): {tally_text(tally(balls, marks), len(balls))}")
        settings = defaultdict(list)
        for key, r in balls:
            settings[(round(f(r, "target_rpm")), round(f(r, "launch_deg"), 1), f(r, "head_trim_deg", 0))].append(
                (key, r))
        for s in sorted(settings):
            b = settings[s]
            print(f"    {s[0]:5d} / {s[1]:5.1f}" + (f" aim{s[2]:+.2f}" if s[2] else "") +
                  f": {tally_text(tally(b, marks), len(b))}")
        searched = [(k, r) for k, r in balls if (r.get("search") or "").startswith(("hood:", "rpm:"))]
        # Hood searches together; RPM searches by launch angle, as the backward search steps the hood steeper
        groups = defaultdict(list)
        for k, r in searched:
            by_rpm = r["search"].startswith("rpm:")
            groups[(by_rpm, round(f(r, "launch_deg"), 1) if by_rpm else None)].append((k, r))
        for (by_rpm, launch) in sorted(groups, key=lambda g: (g[0], -(g[1] or 0))):
            group = groups[(by_rpm, launch)]
            value = (lambda r: f(r, "target_rpm")) if by_rpm else (lambda r: f(r, "launch_deg"))
            ins = sorted(value(r) for k, r in group if marks.get(k) == "IN")
            shorts = sorted(value(r) for k, r in group if "SHORT" in outcomes(marks.get(k)))
            longs = sorted(value(r) for k, r in group if "LONG" in outcomes(marks.get(k)))
            unit, fmt = (f"rpm at launch {launch:.1f}", "{:.0f}") if by_rpm else ("deg launch", "{:.1f}")
            line = "    search (" + unit + "): "
            if ins:
                lo, hi = ins[0], ins[-1]
                line += f"scored {fmt.format(lo)}-{fmt.format(hi)}, middle {fmt.format((lo + hi) / 2)}"
            else:
                line += "nothing scored"
            if shorts:
                line += "; short at " + ", ".join(fmt.format(v) for v in shorts)
            if longs:
                line += "; long at " + ", ".join(fmt.format(v) for v in longs)
            print(line)

    if moving:
        print("\nMOVING PASSES (the tables' shot, fired at the pass's middle): outcome, mean speeds m/s")
        passes = defaultdict(list)
        for key, r in moving.items():
            passes[(r["mode"], r.get("spot") or "-", r["pass"])].append((key, r))
        for (mode, where, kind) in sorted(passes):
            b = passes[(mode, where, kind)]
            print(f"  {mode:8} {where:4} {kind:12} {tally_text(tally(b, marks), len(b))}   toward "
                  f"{mean(f(r, 'toward_mps') for _, r in b):+.2f} sideways {mean(f(r, 'sideways_mps') for _, r in b):+.2f}"
                  f"  rpm err {mean(f(r, 'rpm_at_feed') - f(r, 'target_rpm') for _, r in b):+.0f}"
                  f"  head err {mean(f(r, 'head_err_deg') for _, r in b):+.2f}")

    # Confounders: each ball less the mean of the balls with the same mode, spot and setting, so the setting
    # itself drops out; then the mean of those differences per outcome
    checks = [
        ("head err deg", lambda r: f(r, "head_err_deg")),
        ("rpm@feed-target", lambda r: f(r, "rpm_at_feed") - f(r, "target_rpm")),
        ("hood@feed-launch", lambda r: f(r, "hood_at_feed_deg") - f(r, "launch_deg")),
        ("battery V", lambda r: f(r, "battery_v")),
    ]
    groups = defaultdict(list)
    for key, r in standing.items():
        groups[(r["mode"], r.get("spot"), round(f(r, "target_rpm")), round(f(r, "launch_deg"), 1),
                f(r, "head_trim_deg", 0))].append((key, r))
    rows = defaultdict(lambda: defaultdict(list))  # outcome -> check -> differences
    raw = defaultdict(lambda: defaultdict(list))
    for balls in groups.values():
        if len(balls) < 2:
            continue
        for name, get in checks:
            m = mean(get(r) for _, r in balls)
            for key, r in balls:
                for o in outcomes(marks.get(key)):
                    rows[o][name].append(get(r) - m)
                    raw[o][name].append(get(r))
    print("\nCONFOUNDERS, standing balls in settings fired 2+ times: mean difference from the setting's mean"
          " (raw mean in brackets)")
    print("  expect: LEFT/RIGHT misses with a heading error that way (left +), SHORT with low rpm or launch off,"
          " LONG the reverse")
    print(f"  {'outcome':9} {'n':>3}  " + "  ".join(f"{name:>22}" for name, _ in checks))
    for o in OUTCOMES:
        if not rows[o]:
            continue
        n = len(rows[o][checks[0][0]])
        print(f"  {o:9} {n:3}  " + "  ".join(
            f"{mean(rows[o][name]):+9.2f} ({mean(raw[o][name]):+9.2f})" for name, _ in checks))
    odd = []
    for key, r in standing.items():
        why = []
        if abs(f(r, "head_err_deg", 0)) > 1.0:
            why.append(f"head err {f(r, 'head_err_deg'):+.1f} deg")
        if abs(f(r, "rpm_at_feed", 0) - f(r, "target_rpm", 0)) > 60:
            why.append(f"rpm {f(r, 'rpm_at_feed') - f(r, 'target_rpm'):+.0f} off")
        if abs(f(r, "hood_at_feed_deg", 0) - f(r, "launch_deg", 0)) > 0.6:
            why.append(f"hood {f(r, 'hood_at_feed_deg') - f(r, 'launch_deg'):+.1f} deg off")
        if why:
            odd.append((key, r, why))
    if odd:
        print("\n  balls fired off their setting (the search may have been misled by these):")
        for key, r, why in sorted(odd):
            print(f"    {key[0]} #{key[1]:<3} {r['mode']:8} {r.get('spot', ''):4} {marks.get(key, '-'):12} "
                  + ", ".join(why))


if __name__ == "__main__":
    main()
