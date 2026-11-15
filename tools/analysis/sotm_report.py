#!/usr/bin/env python3
"""SOTM Tuning report: how moving balls did, and fitted corrections for the moving-shot model.

  python3 sotm_report.py [SotmTuning.csv] [--session S] [--pull]

--pull copies /sdcard/FIRST/SotmTuning.csv off the Control Hub first (into logs/, with a timestamped backup).
Plain Python, no numpy. Reads every marked ball (the last mark of each counts) and prints:

  0. The moving spots: what each spot's search found (the range trim that scores, the aim trim), and straight
     fits of those against speed, with one intercept per distance band (the standing error there):
     -> ShootOnTheMoveConstants.SOTM_HOOD_DEG_PER_TOWARD_MPS, SOTM_RPM_PER_TOWARD_MPS,
        SOTM_HEADING_DEG_PER_SIDEWAYS_MPS (and the BACKWARD_ ones), plus FEED_DELAY_S from how the aim trim
        changes with distance. This is the main result, the moving equivalent of Shot Tuning's tables.
  1. Hit rates by pass kind, by speed and by gates, and how many passes fired inside their window.
  2. Release delay: the time after the feed at which the robot's heading best explains the left/right misses.
     Compare with ShootOnTheMoveConstants.FEED_DELAY_S (the aim's lead).
  3. Sideways: whether the shooter's turn for sideways speed is too much or too little ->
     ShootOnTheMoveConstants.SOTM_SIDEWAYS_GAIN.
  4. Range: whether moving toward / away pushes balls long / short -> ShootOnTheMoveConstants.SOTM_TOWARD_GAIN,
     and how acceleration at the feed moves the ball.

Sections 2-4 use only the check passes' balls (one ball each, no search trims), as a cross-check.
Marks: LEFT/RIGHT are as seen from behind the shot (left = anticlockwise from above). Those fits are ordered
probit models (miss one way / in / miss the other way) on a latent miss in degrees (sideways) or arbitrary
units (range), so they work with only marks, no measured miss distances.
"""
import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import sys
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
HUB = "192.168.43.1:5555"


def pull(dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    if os.path.exists(dest):
        backup = os.path.join(HERE, "logs", "backup", "SotmTuning_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.dirname(backup), exist_ok=True)
        shutil.copy(dest, backup)
    subprocess.run(["adb", "-s", HUB, "pull", "/sdcard/FIRST/SotmTuning.csv", dest], check=True)


def fnum(row, key):
    try:
        v = float(row.get(key, ""))
        return v if math.isfinite(v) else None
    except ValueError:
        return None


FOUND_COLUMNS = ["session", "ball", "kind", "mode", "plan_key", "range_kind", "centre", "lo", "hi",
                 "head_trim_deg", "balls", "scored"]


def load(path, session=None):
    """(balls, skipped keys, found): found is each spot's latest FOUND row, if no ball of that spot came after it
    (a changed mark that undid the result)."""
    shots = {}
    marks = {}
    skipped = set()
    found = {}
    last_ball = {}
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        for i, fields in enumerate(reader):
            if len(fields) < 3 or (session and fields[0] != session):
                continue
            kind = fields[2]
            if kind == "FOUND":
                row = dict(zip(FOUND_COLUMNS, fields))
                found[row["plan_key"]] = (i, row)
                continue
            row = dict(zip(header, fields))
            key = (row["session"], row["ball"])
            if kind == "SKIP":
                skipped.add(row.get("plan_key", ""))
                continue
            if row.get("plan_key"):
                last_ball[row["plan_key"]] = i
            if kind == "SHOT":
                shots[key] = row
            elif kind == "UNSEEN":
                marks.pop(key, None)  # Y on the robot: nobody saw it, so it counts as unmarked
            else:
                marks[key] = kind  # Later marks replace earlier ones
    balls = []
    for key, row in shots.items():
        row = dict(row)
        row["mark"] = marks.get(key)
        balls.append(row)
    found = {k: row for k, (i, row) in found.items() if i > last_ball.get(k, -1)}
    return balls, skipped, found


def parse_mark(m):
    """(side, range): side +1 left / -1 right / 0 none or in; range +1 long / -1 short / 0. None if unmarked."""
    if m is None:
        return None
    if m in ("IN", "BOUNCE"):
        return {"in": True, "bounce": m == "BOUNCE", "side": 0, "range": 0, "has_side": True, "has_range": True}
    parts = set(re.split(r"[+_ ]", m))
    side = 1 if "LEFT" in parts else -1 if "RIGHT" in parts else 0
    rng = 1 if "LONG" in parts else -1 if "SHORT" in parts else 0
    # A miss marked only long/short says nothing certain about the side, and the other way round
    return {"in": False, "bounce": False, "side": side, "range": rng, "has_side": side != 0, "has_range": rng != 0}


# ---- Small numerics

def phi(z):
    return 0.5 * math.erfc(-z / math.sqrt(2))


def nelder_mead(f, x0, step=0.5, iters=2000, tol=1e-7):
    n = len(x0)
    pts = [list(x0)]
    for i in range(n):
        p = list(x0)
        p[i] += step if p[i] == 0 else step * max(1.0, abs(p[i]))
        pts.append(p)
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        centroid = [sum(p[i] for p in pts[:-1]) / n for i in range(n)]
        refl = [centroid[i] + (centroid[i] - pts[-1][i]) for i in range(n)]
        fr = f(refl)
        if fr < vals[0]:
            exp = [centroid[i] + 2 * (centroid[i] - pts[-1][i]) for i in range(n)]
            fe = f(exp)
            pts[-1], vals[-1] = (exp, fe) if fe < fr else (refl, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = refl, fr
        else:
            con = [centroid[i] + 0.5 * (pts[-1][i] - centroid[i]) for i in range(n)]
            fc = f(con)
            if fc < vals[-1]:
                pts[-1], vals[-1] = con, fc
            else:
                for j in range(1, n + 1):
                    pts[j] = [pts[0][i] + 0.5 * (pts[j][i] - pts[0][i]) for i in range(n)]
                    vals[j] = f(pts[j])
    best = min(range(n + 1), key=lambda i: vals[i])
    return pts[best], vals[best]


def ordered_probit(data, n_coef, start=None):
    """data: list of (features, outcome -1/0/+1). Latent = b0 + sum(b_i x_i) + N(0, 1) (units of the noise);
    in when |latent| < w. Returns (coefs, w, -loglik)."""
    if len(data) < n_coef + 2:
        return None

    def nll(p):
        b, w = p[:-1], abs(p[-1]) + 1e-3
        s = 0.0
        for x, y in data:
            m = b[0] + sum(bi * xi for bi, xi in zip(b[1:], x))
            if y > 0:
                pr = 1 - phi(w - m)
            elif y < 0:
                pr = phi(-w - m)
            else:
                pr = phi(w - m) - phi(-w - m)
            s -= math.log(max(pr, 1e-12))
        # Weak prior keeps coefficients finite when the data separate perfectly
        return s + 0.01 * sum(bi * bi for bi in b)

    x0 = start or ([0.0] * (n_coef + 1) + [1.0])
    best, val = nelder_mead(nll, x0)
    best, val = nelder_mead(nll, best, step=0.1)
    return best[:-1], abs(best[-1]) + 1e-3, val


def bootstrap(data, n_coef, fn, reps=60):
    import random
    rnd = random.Random(1)
    out = []
    for _ in range(reps):
        sample = [data[rnd.randrange(len(data))] for _ in data]
        r = ordered_probit(sample, n_coef)
        if r:
            out.append(fn(r))
    out.sort()
    if len(out) < 10:
        return None
    return out[len(out) // 10], out[len(out) * 9 // 10]


def shot_table():
    """{dist: [...], vel: [...], rpm: [[...]]} from ShotTable.java, for the RPM's slope with speed."""
    path = os.path.join(REPO, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooting/ShotTable.java")
    try:
        text = open(path).read()
    except OSError:
        return None

    def arr(name):
        m = re.search(name + r"\s*=\s*\{([^;]*?)\};", text, re.S)
        if not m:
            return None
        body = m.group(1)
        if "{" in body:
            return [[float(v) if v.strip() not in ("NaN", "Double.NaN") else float("nan")
                     for v in row.split(",") if v.strip()] for row in re.findall(r"\{([^{}]*)\}", body)]
        return [float(v) for v in body.split(",") if v.strip()]

    return {"dist": arr("DIST_M"), "vel": arr("VEL_MPS"), "rpm": arr(r"double\[\]\[\] RPM")}


def rpm_slope(table, dist):
    """d(target RPM)/d(speed toward the Cell) at speed 0, from the table, nearest distance row."""
    if not table or not table["rpm"]:
        return None
    di = min(range(len(table["dist"])), key=lambda i: abs(table["dist"][i] - dist))
    vel = table["vel"]
    v0 = min(range(len(vel)), key=lambda i: abs(vel[i]))
    row = table["rpm"][di] if len(table["rpm"]) == len(table["dist"]) else None
    if row is None or v0 + 1 >= len(row) or v0 < 1:
        return None
    a, b = row[v0 - 1], row[v0 + 1]
    if not (math.isfinite(a) and math.isfinite(b)):
        return None
    return (b - a) / (vel[v0 + 1] - vel[v0 - 1])


# ---- Moving spots

def least_squares(xs, ys):
    """Ordinary least squares, y = X b. Returns (b, standard errors, residual sd) or None."""
    n, k = len(xs), len(xs[0]) if xs else 0
    if n <= k or k == 0:
        return None
    a = [[sum(x[i] * x[j] for x in xs) for j in range(k)] for i in range(k)]
    v = [sum(x[i] * y for x, y in zip(xs, ys)) for i in range(k)]
    inv = invert(a)
    if inv is None:
        return None
    b = [sum(inv[i][j] * v[j] for j in range(k)) for i in range(k)]
    res = [y - sum(bi * xi for bi, xi in zip(b, x)) for x, y in zip(xs, ys)]
    s2 = sum(r * r for r in res) / (n - k)
    return b, [math.sqrt(max(0.0, s2 * inv[i][i])) for i in range(k)], math.sqrt(s2)


def invert(a):
    n = len(a)
    m = [list(row) + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        piv = m[c][c]
        m[c] = [x / piv for x in m[c]]
        for r in range(n):
            if r != c and m[r][c] != 0:
                f = m[r][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [row[n:] for row in m]


def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def band_of(mode, dist):
    # SotmPath's distance bands, to the Cell's aim point
    edges = [0.7] if mode == "BACKWARD" else [1.1, 1.35]
    i = sum(dist >= e for e in edges)
    names = ["near", "far"] if mode == "BACKWARD" else ["near", "mid", "far"]
    return names[i]


def spot_rows(balls, found):
    by_key = defaultdict(list)
    for b in balls:
        if b.get("search", "").startswith("SEARCH"):
            by_key[b["plan_key"]].append(b)
    rows = []
    for key, fd in found.items():
        bs = by_key.get(key)
        if not bs:
            continue
        dists = []
        for b in bs:
            cx, cy, x, y = (fnum(b, k) for k in ("cell_x_m", "cell_y_m", "f_x_m", "f_y_m"))
            if None not in (cx, cy, x, y):
                dists.append(math.hypot(cx - x, cy - y))
        last = bs[-1]
        rows.append({
            "key": key, "mode": fd["mode"], "range_kind": fd["range_kind"],
            "centre": float(fd["centre"]), "lo": float(fd["lo"]), "hi": float(fd["hi"]),
            "head": float(fd["head_trim_deg"]), "balls": int(fd["balls"]), "scored": int(fd["scored"]),
            "toward": mean(fnum(b, "f_toward_mps") for b in bs), "sideways": mean(fnum(b, "f_sideways_mps") for b in bs),
            "dist": mean(dists) or fnum(last, "aim_dist_m") or 1.0,
            "toward_gain": fnum(last, "toward_gain") or 1.0, "sideways_gain": fnum(last, "sideways_gain") or 1.0,
            "hood_per_toward": fnum(last, "hood_per_toward") or 0.0,
            "rpm_per_toward": fnum(last, "rpm_per_toward") or 0.0,
            "head_per_sideways": fnum(last, "head_per_sideways") or 0.0,
            "feed_delay": fnum(last, "feed_delay_s"),
        })
    for r in rows:
        r["band"] = band_of(r["mode"], r["dist"])
    return rows


def fit_with_bands(rows, yname, xname):
    """y = intercept per distance band + slope * x. Returns (slope, se, {band: intercept}, sd) or None."""
    bands = sorted({r["band"] for r in rows})
    xs = [[1.0 if r["band"] == bd else 0.0 for bd in bands] + [r[xname]] for r in rows]
    ys = [r[yname] for r in rows]
    fit = least_squares(xs, ys)
    if not fit:
        return None
    b, se, sd = fit
    return b[-1], se[-1], dict(zip(bands, b[:-1])), sd


def report_spots(balls, found):
    print("\n0. Moving spots (each spot's search, the moving equivalent of Shot Tuning's)")
    rows = spot_rows(balls, found)
    searching = {b["plan_key"] for b in balls if b.get("search", "").startswith("SEARCH")} - set(found)
    print("   %d spots found, %d searched but not finished" % (len(rows), len(searching)))
    if not rows:
        return
    print("   %-42s %4s %6s %6s %5s %16s %7s %5s" % ("spot", "by", "toward", "side", "dist", "range trim", "aim",
                                                    "in"))
    for r in sorted(rows, key=lambda r: (r["mode"], r["band"], r["toward"] or 0, r["sideways"] or 0)):
        fmt = "%+.0f" if r["range_kind"] == "RPM" else "%+.1f"
        rng = (fmt + " (" + fmt + ".." + fmt + ")") % (r["centre"], r["lo"], r["hi"])
        print("   %-42s %4s %+6.2f %+6.2f %5.2f %16s %+7.2f %2d/%-2d" % (
            r["key"][:42], r["range_kind"], r["toward"] or 0, r["sideways"] or 0, r["dist"], rng, r["head"],
            r["scored"], r["balls"]))
    for mode in ("HOODED", "BACKWARD"):
        mr = [r for r in rows if r["mode"] == mode and r["toward"] is not None and r["sideways"] is not None]
        if not mr:
            continue
        prefix = "SOTM_BACKWARD_" if mode == "BACKWARD" else "SOTM_"
        print("\n   %s, %d spots" % (mode, len(mr)))
        for kind, unit, const, now_key in (("HOOD", "deg", "SOTM_HOOD_DEG_PER_TOWARD_MPS", "hood_per_toward"),
                                           ("RPM", "rpm", prefix + "RPM_PER_TOWARD_MPS", "rpm_per_toward")):
            kr = [r for r in mr if r["range_kind"] == kind]
            if not kr:
                continue
            spread = max(r["toward"] for r in kr) - min(r["toward"] for r in kr)
            fit = fit_with_bands(kr, "centre", "toward") if spread >= 0.3 else None
            if not fit:
                print("   range by %s: %d spots, toward speeds spread %.2f m/s: need 3+ spots spread 0.3+ m/s" % (
                    kind, len(kr), spread))
                continue
            slope, se, icpt, sd = fit
            gain = kr[-1]["toward_gain"]
            now = kr[-1][now_key]
            print("   range by %s: %+.2f %s per m/s toward (+/- %.2f), scatter %.2f %s" % (kind, slope, unit, se, sd, unit))
            print("     at rest (standing error, not moving): " + ", ".join(
                "%s %+.2f %s" % (bd, v, unit) for bd, v in sorted(icpt.items())))
            flag = "" if abs(slope) > 2 * se else "  (within noise: leave it unless more spots agree)"
            print("     -> %s %.3f -> %.3f%s" % (const, now, now + slope / gain, flag))
        spread = max(r["sideways"] for r in mr) - min(r["sideways"] for r in mr)
        fit = fit_with_bands(mr, "head", "sideways") if spread >= 0.3 else None
        if not fit:
            print("   aim: %d spots, sideways speeds spread %.2f m/s: need 3+ spots spread 0.3+ m/s" % (len(mr), spread))
            continue
        slope, se, icpt, sd = fit
        gain = mr[-1]["sideways_gain"]
        now = mr[-1]["head_per_sideways"]
        print("   aim: %+.2f deg anticlockwise per m/s sideways (left +) (+/- %.2f), scatter %.2f deg" % (slope, se, sd))
        print("     at rest (the standing trim): " + ", ".join("%s %+.2f deg" % (bd, v) for bd, v in sorted(icpt.items())))
        flag = "" if abs(slope) > 2 * se else "  (within noise: leave it unless more spots agree)"
        print("     -> %sHEADING_DEG_PER_SIDEWAYS_MPS %.3f -> %.3f%s" % (prefix, now, now + slope / gain, flag))
        # A wrong release delay misses sideways by speed x delay error / distance; a wrong turn by speed alone
        dists = [r["dist"] for r in mr]
        if max(dists) - min(dists) >= 0.25 and len(mr) >= 6:
            xs = [[1.0, r["sideways"], math.degrees(r["sideways"] / r["dist"])] for r in mr]
            f2 = least_squares(xs, [r["head"] for r in mr])
            if f2:
                b, se2, _ = f2
                delay = mr[-1]["feed_delay"]
                print("     split by distance: %+.2f deg per m/s (turn) and %+.3f s release delay error (+/- %.3f)"
                      % (b[1], -b[2], se2[2]))
                if delay is not None:
                    flag = "" if abs(b[2]) > 2 * se2[2] else "  (within noise: use the one slope above)"
                    print("     -> OR, instead of the one slope above: FEED_DELAY_S %.3f -> %.3f and %sHEADING_DEG_"
                          "PER_SIDEWAYS_MPS %.3f -> %.3f%s. Holds better across distances; not both" % (
                              delay, delay - b[2], prefix, now, now + b[1] / gain, flag))

# ---- Report

def pct(a, b):
    return "%3d%%" % round(100 * a / b) if b else "  - "


def table_by(balls, keyf, title):
    groups = defaultdict(list)
    for b in balls:
        groups[keyf(b)].append(b)
    print("\n%s" % title)
    print("  %-34s %5s %5s %5s %5s %5s %5s %5s" % ("", "balls", "in", "short", "long", "left", "right", "bnc"))
    for k in sorted(groups, key=str):
        g = groups[k]
        m = [b["pm"] for b in g]
        print("  %-34s %5d %5s %5d %5d %5d %5d %5d" % (
            k, len(g), pct(sum(x["in"] for x in m), len(m)), sum(x["range"] < 0 for x in m),
            sum(x["range"] > 0 for x in m), sum(x["side"] > 0 for x in m), sum(x["side"] < 0 for x in m),
            sum(x["bounce"] for x in m)))


def head_err_at(b, tau):
    """Heading error (deg, target minus heading) tau s after the feed, linear between the feed and the dip."""
    e0, e1, dt = fnum(b, "f_head_err_deg"), fnum(b, "d_head_err_deg"), fnum(b, "dip_after_feed_s")
    if e0 is None or e1 is None or not dt:
        return None
    return e0 + (e1 - e0) * tau / dt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default=os.path.join(HERE, "logs", "SotmTuning.csv"))
    ap.add_argument("--session")
    ap.add_argument("--pull", action="store_true")
    args = ap.parse_args()
    if args.pull:
        pull(args.csv)
    balls, skipped, found = load(args.csv, args.session)
    marked = []
    for b in balls:
        b["pm"] = parse_mark(b["mark"])
        if b["pm"] is not None:
            marked.append(b)
    moving = [b for b in marked if b.get("plan_key")]
    print("%s: %d balls, %d marked, %d from passes; %d passes skipped" % (
        os.path.basename(args.csv), len(balls), len(marked), len(moving), len(skipped)))
    if not marked:
        return
    report_spots(balls, found)

    def speed_bin(b):
        vx, vy = fnum(b, "f_vx_mps") or 0, fnum(b, "f_vy_mps") or 0
        s = math.hypot(vx, vy)
        return "%.1f-%.1f m/s" % (math.floor(s / 0.2) * 0.2, math.floor(s / 0.2) * 0.2 + 0.2)

    table_by(marked, lambda b: b["mode"] + " " + (b["path"] or "standing (RB)"), "1. By pass kind")
    table_by(marked, lambda b: b["mode"] + " " + speed_bin(b), "   By speed at the feed")
    table_by(moving, lambda b: "gates " + b["gates"], "   By gates")
    table_by(moving, lambda b: "spot search" if b.get("search", "").startswith("SEARCH") else "check", "   Spots or checks")
    late = sorted(fnum(b, "feed_s") - fnum(b, "plan_fire_s") for b in moving
                  if fnum(b, "feed_s") is not None and fnum(b, "plan_fire_s") is not None)
    dips = sorted(fnum(b, "dip_after_feed_s") for b in balls if fnum(b, "dip_after_feed_s") is not None)
    if late:
        print("\n   Feed after the window opened: median %.2f s, 90%% within %.2f s" % (
            late[len(late) // 2], late[int(len(late) * 0.9)]))
    if dips:
        print("   Feed to the flywheels' dip: median %.2f s (%.2f-%.2f); the dip lags the ball by ~0.1-0.17 s" % (
            dips[len(dips) // 2], dips[0], dips[-1]))

    # The spots' balls are fired with search trims that move them on purpose; 2-4 use the rest
    marked = [b for b in marked if not b.get("search", "").startswith("SEARCH")]
    print("\n   Sections 2-4: %d check and standing balls" % len(marked))

    # 2. Release delay from the left/right marks
    print("\n2. Release delay (left/right marks against the heading error after the feed)")
    side = [b for b in marked if b["pm"]["has_side"] or b["pm"]["in"]]
    best = None
    for i in range(0, 11):
        tau = 0.05 * i
        data = []
        for b in side:
            e = head_err_at(b, tau)
            if e is None:
                continue
            # Heading error = target - heading: positive means the robot points clockwise of the shot, ball right
            data.append(([-e], b["pm"]["side"]))
        r = ordered_probit(data, 1)
        if not r:
            continue
        coefs, w, nll = r
        print("   %.2f s after the feed: fit -log L %.1f, ball follows the heading x%.2f (noise units/deg)" % (
            tau, nll, coefs[1]))
        if best is None or nll < best[1]:
            best = (tau, nll)
    if best:
        print("   -> best %.2f s. The aim leads by FEED_DELAY_S; if they differ by more than ~0.05 s with 20+ side-"
              "marked balls, set FEED_DELAY_S to it" % best[0])

    # 3. Sideways gain
    print("\n3. Sideways correction (SOTM_SIDEWAYS_GAIN)")
    tau = best[0] if best else 0.3
    data = []
    for b in side:
        e = head_err_at(b, tau)
        t = fnum(b, "f_turning_deg")
        if e is None or t is None:
            continue
        data.append(([-e, t], b["pm"]["side"]))
    r = ordered_probit(data, 2)
    if r:
        coefs, w, _ = r
        k_head, k_turn = coefs[1], coefs[2]
        if abs(k_head) > 1e-3:
            c = k_turn / k_head  # Ball's sideways miss per degree of turn, in degrees
            gain_now = fnum(side[0], "sideways_gain") or 1.0
            ci = bootstrap(data, 2, lambda rr: rr[0][2] / rr[0][1] if abs(rr[0][1]) > 1e-3 else 0.0)
            print("   ball misses %+.2f deg toward the turn per degree of turn%s" % (
                c, "" if not ci else " (80%% range %+.2f to %+.2f)" % ci))
            print("   -> SOTM_SIDEWAYS_GAIN %.2f -> %.2f (over-turned misses with the turn: turn less)" % (
                gain_now, gain_now * (1 - c)))
        else:
            print("   heading doesn't explain the side misses yet: more balls needed")
    else:
        print("   too few balls marked for side")

    # 4. Range
    print("\n4. Range (SOTM_TOWARD_GAIN, acceleration)")
    rng = [b for b in marked if b["pm"]["has_range"] or b["pm"]["in"]]
    data = []
    for b in rng:
        tw, rpm, target = fnum(b, "f_toward_mps"), fnum(b, "d_rpm"), fnum(b, "f_target_rpm")
        vx, vy, ax, ay = (fnum(b, k) for k in ("f_vx_mps", "f_vy_mps", "f_ax_mps2", "f_ay_mps2"))
        cx, cy, x, y = (fnum(b, k) for k in ("cell_x_m", "cell_y_m", "f_x_m", "f_y_m"))
        if None in (tw, rpm, target, ax, ay, cx, cy, x, y):
            continue
        d = math.hypot(cx - x, cy - y) or 1
        a_toward = (ax * (cx - x) + ay * (cy - y)) / d
        data.append(([tw, a_toward, (rpm - target) / 100.0], b["pm"]["range"]))
    r = ordered_probit(data, 3)
    if r:
        coefs, w, _ = r
        k_v, k_a, k_rpm = coefs[1:4]
        print("   per m/s toward the Cell %+.2f, per m/s^2 toward %+.2f, per 100 rpm fast %+.2f (noise units, + is long)"
              % (k_v, k_a, k_rpm))
        if k_rpm > 0.05:
            rpm_per_mps = 100 * k_v / k_rpm  # Extra RPM-equivalent the ball gets per m/s closing
            dist = sorted(fnum(b, "f_lip_m") or 1 for b in rng)[len(rng) // 2]
            slope = rpm_slope(shot_table(), dist)
            print("   closing at 1 m/s acts like %+.0f rpm fast" % rpm_per_mps)
            print("   accelerating toward at 1 m/s^2 acts like %+.0f rpm fast" % (100 * k_a / k_rpm))
            if slope and abs(slope) > 1:
                gain_now = fnum(rng[0], "toward_gain") or 1.0
                print("   table RPM slope %.0f rpm per m/s at %.2f m -> SOTM_TOWARD_GAIN %.2f -> %.2f (rough: the "
                      "table also moves the hood)" % (slope, dist, gain_now, gain_now - rpm_per_mps / slope))
        else:
            print("   RPM doesn't explain the range misses yet (needs balls both short and long): more balls needed")
    else:
        print("   too few balls marked for range")


if __name__ == "__main__":
    sys.exit(main())
