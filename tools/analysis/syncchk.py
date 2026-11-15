#!/usr/bin/env python3
"""syncchk.py LOG.rlog [--offset S]

Checks a log's clock offset against the field webcam: the robot's speed from odometry should rise and fall with
the motion the webcam sees on the field. Prints the best shift (seconds to add to the offset) and how well they
match (r). A right offset gives a shift near 0 with r well above the others; a log from another hub boot gives no
clear peak. Exit code 0 when the offset holds (|shift| <= 0.5 s, r >= 0.35)."""
import sys, os, math
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rlogparse import parse
import replay

STEP = 0.2


def robot_speed(d, grid):
    R = 'RealOutputs/'
    vx, vy, w = (d.get(R + 'Odometry/Velocity/' + k, []) for k in ('XInchesPerSec', 'YInchesPerSec', 'HeadingDegreesPerSec'))
    if not vx:
        return None
    n = min(len(vx), len(vy))  # One loop can log one but not the other at the end
    t = np.array([a for a, _ in vx[:n]])
    s = np.hypot([b for _, b in vx[:n]], [b for _, b in vy[:n]]) * 0.0254
    if len(w) >= n:
        s = s + 0.25 * np.abs(np.radians([b for _, b in w[:n]]))
    return np.interp(grid, t, s, left=0, right=0)


def webcam_motion(a, b, crop):
    out_t, out_m = [], []
    t = a
    while t < b:
        dur = min(20.0, b - t)
        try:
            fr, times = replay.frames(t, dur, crop, fps=1 / STEP)
        except SystemExit:
            fr, times = [], []
        prev = None
        for f, tt in zip(fr, times):
            g = cv2.cvtColor(cv2.resize(f, (192, int(192 * f.shape[0] / f.shape[1]))), cv2.COLOR_BGR2GRAY).astype(np.int16)
            if prev is not None:
                out_t.append(tt)
                out_m.append(float(np.mean(np.abs(g - prev) > 18)))
            prev = g
        t += dur
    return np.array(out_t), np.array(out_m)


def check(path, off):
    d, _ = parse(path)
    ts = [x for k, v in d.items() if k.startswith('RealOutputs/') and v for x in (v[0][0], v[-1][0])]
    a, b = min(ts), max(ts)
    try:
        crop = replay.resolve_crop('field', a + off)
    except Exception:
        crop = None
    wt, wm = webcam_motion(a + off - 4, b + off + 4, crop)
    if len(wt) < 30:
        return None, None, 'no webcam video'
    grid = np.arange(a, b, STEP)
    sp = robot_speed(d, grid)
    if sp is None or sp.std() < 0.02:
        return None, None, 'robot hardly moved'
    best = (-2, 0)
    rz = None
    for shift in np.arange(-4, 4.01, STEP):
        m = np.interp(grid + off + shift, wt, wm)
        if m.std() == 0:
            continue
        r = float(np.corrcoef(sp, m)[0, 1])
        if abs(shift) < 1e-6:
            rz = r
        if r > best[0]:
            best = (r, shift)
    return best[1], best[0], 'r at 0: %s' % ('%.2f' % rz if rz is not None else '-')


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    if '--offset' in sys.argv:
        off = float(sys.argv[sys.argv.index('--offset') + 1])
    else:
        off = float(open(path + '.clock').read().split()[0])
    shift, r, note = check(path, off)
    if shift is None:
        print('%s: %s' % (os.path.basename(path), note))
        sys.exit(2)
    ok = abs(shift) <= 0.5 and r >= 0.35
    print('%s: shift %+.1f s, r %.2f (%s) %s' % (os.path.basename(path), shift, r, note, 'OK' if ok else 'NO'))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
