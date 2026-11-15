#!/usr/bin/env python3
"""balltrack.py START DURATION OUT.jpg [crop] [--debug]

Finds the balls flying in the field webcam video and draws their paths on one picture: every detection a dot
coloured by time (blue early -> red late), the time written every 0.2 s, and the Cell outlines from camera.json.
Prints each track: start/end time, where it went, and where it ended relative to the Cells.

A ball is a small yellow-green blob that differs from the background (the median of the window, so a ball resting
on the mat does not count). START as replay.py takes it; crop defaults to 'hive'."""
import sys, os, json
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay, camera


def detect(frames):
    """Per frame: candidate ball blobs (x, y, area). Background for frame i is the median of its neighbours about
    half a second either side (not the frames right next to it, where the ball itself still is), after evening out
    the brightness, so exposure drift, people and the moving robot change slowly enough to drop out. The change
    test takes the largest of the three colour channels: yellow on a white wall differs mostly in blue."""
    n = len(frames)
    level = np.array([float(np.median(f[::4, ::4])) for f in frames])
    gain = np.median(level) / np.maximum(level, 1)
    out = []
    stats = {'candidates': 0, 'crowded_frames': 0}
    for i in range(n):
        ref = [j for j in (i - 9, i - 7, i - 5, i + 5, i + 7, i + 9) if 0 <= j < n]
        if len(ref) < 3:
            ref = [j for j in range(max(0, i - 9), min(n, i + 10)) if abs(j - i) >= 3]
        bgi = np.median(np.stack([frames[j].astype(np.float32) * gain[j] for j in ref]), axis=0)
        cur = frames[i].astype(np.float32) * gain[i]
        moving = np.abs(cur - bgi).max(axis=2) > 28
        hsv = cv2.cvtColor(frames[i], cv2.COLOR_BGR2HSV)
        h, s_, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        # Ball pixels measured on 5 Nov: hue 22-37, saturation 50-128 (blur washes it out), value 164-236
        yellow = (h >= 21) & (h <= 40) & (s_ >= 45) & (v >= 130)
        m = (yellow & moving).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        k, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
        dets = []
        for c in range(1, k):
            x, y, w, hh, a = st[c]
            if 4 <= a <= 400 and w <= 3 * hh + 3 and hh <= 3 * w + 3:
                dets.append((float(cen[c][0]), float(cen[c][1]), int(a)))
        # A frame full of candidates is a lighting change or a person, not balls
        if len(dets) > 12:
            stats['crowded_frames'] += 1
            dets = []
        stats['candidates'] += len(dets)
        out.append(dets)
    detect.stats = stats
    return out, np.median(frames[::max(1, n // 15)], axis=0).astype(np.uint8)


def flight_fit(pts):
    """Fits x = a + b t + c t^2 (drag), y = d + e t + f t^2. Returns (rms px, f) or None."""
    p = np.array([q[:3] for q in pts], float)
    t = p[:, 0] - p[0, 0]
    if len(p) < 5:
        return None
    A = np.stack([np.ones_like(t), t, t * t], axis=1)
    cx, rx, _, _ = np.linalg.lstsq(A, p[:, 1], rcond=None)
    cy, ry, _, _ = np.linalg.lstsq(A, p[:, 2], rcond=None)
    res = np.concatenate([A @ cx - p[:, 1], A @ cy - p[:, 2]])
    return float(np.sqrt(np.mean(res ** 2))), float(cy[2]), cx, cy


def link(dets, times, max_jump, min_rise):
    """Greedy nearest-neighbour tracks with a constant-velocity prediction, allowing two missed frames; kept only
    if they look like a flight: they move, rise or fall, and follow a smooth parabola."""
    tracks = []
    for i, ds in enumerate(dets):
        used = set()
        for tr in sorted([t for t in tracks if t['open']], key=lambda t: -len(t['pts'])):
            gap = i - tr['last']
            if gap > 3:
                tr['open'] = False
                continue
            lx, ly = tr['pts'][-1][1:3]
            if len(tr['pts']) >= 2:
                pt, px, py = tr['pts'][-2][:3]
                dt = max(1e-3, tr['pts'][-1][0] - pt)
                lx += (lx - px) / dt * (times[i] - tr['pts'][-1][0])
                ly += (ly - py) / dt * (times[i] - tr['pts'][-1][0])
                lim = max_jump * 0.5 * gap
            else:
                lim = max_jump * gap
            best, bd = None, lim
            for j, d in enumerate(ds):
                if j in used:
                    continue
                dd = np.hypot(d[0] - lx, d[1] - ly)
                if dd < bd:
                    best, bd = j, dd
            if best is not None:
                used.add(best)
                tr['pts'].append((times[i], ds[best][0], ds[best][1], ds[best][2]))
                tr['last'] = i
        for j, d in enumerate(ds):
            if j not in used:
                tracks.append({'pts': [(times[i], d[0], d[1], d[2])], 'last': i, 'open': True})
    good = []
    why = {'short': 0, 'flat_or_slow': 0, 'not_a_parabola': 0}
    for tr in tracks:
        pts = tr['pts']
        if len(pts) < 5:
            why['short'] += 1
            continue
        p = np.array([q[:3] for q in pts])
        steps = np.hypot(np.diff(p[:, 1]), np.diff(p[:, 2]))
        if np.ptp(p[:, 2]) < min_rise or np.median(steps) < 3:
            why['flat_or_slow'] += 1
            continue
        # The ball lands, bounces or meets another blob: keep the longest opening stretch that is a flight
        kept = None
        for k in range(len(pts), 4, -1):
            q = np.array([x[:3] for x in pts[:k]])
            rms, acc = flight_fit(pts[:k])[:2]
            # Image y grows downward: gravity makes the y acceleration positive
            if rms <= 2.5 + 0.02 * np.ptp(q[:, 2]) and acc >= -50 and np.ptp(q[:, 2]) >= min_rise:
                kept = pts[:k]
                break
        if kept is None:
            why['not_a_parabola'] += 1
            if os.environ.get('BT_DEBUG'):
                rms, acc = flight_fit(pts)[:2]
                print('  rejected parabola rms %.1f acc %.0f n %d %s' % (rms, acc, len(p), replay_time(p[0, 0])))
            continue
        good.append(kept)
    link.why = why
    return good


def cells_in_crop(start, crop, size):
    w, h, x, y = map(float, crop.split(':'))
    pos = camera.position(start)
    W, H = size[1] / w, size[0] / h  # full picture size in crop pixels
    out = []
    for c in pos['cells']:
        bx, by, bw, bh = c['box']
        out.append((c['colour'], c['upper'], [(bx - x) * W, (by - y) * H, bw * W, bh * H]))
    return out


def describe(tr, cells):
    t0, x0, y0 = tr[0][:3]
    t1, x1, y1 = tr[-1][:3]
    s = '%s-%s  %d pts  (%.0f,%.0f)->(%.0f,%.0f)' % (replay_time(t0), replay_time(t1)[6:], len(tr), x0, y0, x1, y1)
    top = min(tr, key=lambda q: q[2])
    s += '  apex y %.0f at %s' % (top[2], replay_time(top[0])[6:])
    for colour, upper, (bx, by, bw, bh) in cells:
        if bx <= x1 <= bx + bw and by <= y1 <= by + bh:
            s += '  ends in %s %s box' % ('upper' if upper else 'lower', colour)
    return s


def replay_time(t):
    import datetime
    d = datetime.datetime.fromtimestamp(t)
    return d.strftime('%H:%M:%S.') + '%03d' % (d.microsecond // 1000)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) < 3:
        sys.exit(__doc__)
    start, dur, out = replay.parse_start(args[0]), float(args[1]), args[2]
    crop = replay.resolve_crop(args[3] if len(args) > 3 else 'hive', start)
    fr, times = replay.frames(start, dur, crop)
    if not len(fr):
        sys.exit('no frames')
    dets, bg = detect(fr)
    jump = max(30.0, fr.shape[2] / 10.0)
    tracks = link(dets, times, jump, fr.shape[1] * 0.05)
    cells = cells_in_crop(start, crop, fr.shape[1:3])
    img = cv2.addWeighted(bg, 0.75, np.full_like(bg, 255), 0.25, 0)
    for colour, upper, (bx, by, bw, bh) in cells:
        cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)),
                      (255, 0, 0) if colour == 'blue' else (0, 0, 255), 1)
    span = max(1e-3, times[-1] - times[0])
    if '--debug' in sys.argv:
        for i, ds in enumerate(dets):
            for d in ds:
                cv2.circle(img, (int(d[0]), int(d[1])), 2, (128, 128, 128), -1)
    for n, tr in enumerate(tracks):
        last_lab = -1
        for k, (t, x, y, a) in enumerate(tr):
            f = (t - times[0]) / span
            col = (int(255 * (1 - f)), 60, int(255 * f))
            if k:
                cv2.line(img, (int(tr[k - 1][1]), int(tr[k - 1][2])), (int(x), int(y)), col, 1)
            cv2.circle(img, (int(x), int(y)), 4, col, -1)
            lab = int((t - times[0]) / 0.2)
            if lab != last_lab:
                cv2.putText(img, replay_time(t)[6:11], (int(x) + 6, int(y) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                            (0, 0, 0), 1)
                last_lab = lab
        print('track %d: %s' % (n, describe(tr, cells)))
    cv2.putText(img, '%s +%.1fs' % (replay_time(times[0]), span), (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
    cv2.imwrite(out, img)
    print(out, len(tracks), 'tracks', detect.stats, link.why)


if __name__ == '__main__':
    main()
