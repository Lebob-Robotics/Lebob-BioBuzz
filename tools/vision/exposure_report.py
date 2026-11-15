#!/usr/bin/env python3
"""Exposure report for a match log: how well each camera saw the AprilTags, and which way to move the exposure.

    tools/vision/exposure_report.py LOG.rlog [LOG2.rlog ...]

Reads a PsiKit .rlog pulled from /sdcard/FIRST/PsiKit (tools/vision/pull_and_report.sh does both). Per camera and per
exposure/gain setting seen in the log it prints frame rate, image brightness (mean grey, % clipped white, % crushed
black), how many frames found a tag, and tag decision margins by distance and by robot speed. Decision margin is the
detector's black/white contrast for a tag: the number that matters. Brightness explains why it is low. Logs from before
the brightness logging still work, without the brightness columns.
"""
import bisect
import collections
import math
import re
import statistics
import struct
import sys

# Margins at home at 5 ms / gain 60, 1-1.5 m: about 140 (VisionSubsystem's COMP note, 15 Nov)
HOME_NEAR_MARGIN = 140
MIN_DECISION_MARGIN = 20  # VisionConstants.MIN_DECISION_MARGIN: below it the pose correction drops the tag
RANGE_BINS_M = [(0, 1.5), (1.5, 2.5), (2.5, 3.5), (3.5, 99)]
SPEED_BINS = [(0, 0.3, "still <0.3 m/s"), (0.3, 1.0, "moving 0.3-1 m/s"), (1.0, 99, "fast >1 m/s")]
SETTLE_S = 0.5  # Ignore frames this soon after an exposure change


def _decode(typ, b):
    try:
        if typ == 'boolean': return b[0] != 0
        if typ == 'int64': return struct.unpack('>q', b)[0]
        if typ == 'float': return struct.unpack('>f', b)[0]
        if typ == 'double': return struct.unpack('>d', b)[0]
        if typ == 'string': return b.decode('utf-8', 'replace')
        if typ == 'int64[]': return list(struct.unpack('>%dq' % (len(b) // 8), b))
        if typ == 'double[]': return list(struct.unpack('>%dd' % (len(b) // 8), b))
        if typ.startswith('struct:') and len(b) % 8 == 0:
            return list(struct.unpack('<%dd' % (len(b) // 8), b))
    except Exception:
        pass
    return b


def parse(path):
    """PsiKit/AdvantageKit .rlog -> {key: [(t, value)]}. Values are logged on change only."""
    data = open(path, 'rb').read()
    i, n, t = 1, len(data), 0.0
    names, types, series = {}, {}, {}
    while i < n:
        kind = data[i]; i += 1
        if kind == 0:
            if i + 8 > n: break
            t = struct.unpack('>d', data[i:i + 8])[0]; i += 8
        elif kind == 1:
            if i + 4 > n: break
            kid, ln = struct.unpack('>hh', data[i:i + 4]); i += 4
            name = data[i:i + ln].decode('utf-8', 'replace'); i += ln
            (tl,) = struct.unpack('>h', data[i:i + 2]); i += 2
            names[kid] = name; types[name] = data[i:i + tl].decode('utf-8', 'replace'); i += tl
        elif kind == 2:
            if i + 4 > n: break
            kid, ln = struct.unpack('>hH', data[i:i + 4]); i += 4
            if i + ln > n: break  # Pulled while still being written
            name = names.get(kid)
            if name is not None:
                series.setdefault(name, []).append((t, _decode(types[name], data[i:i + ln])))
            i += ln
        else:
            break
    return series


class Track:
    """Forward-filled lookup of a change-only series."""
    def __init__(self, series):
        self.ts = [t for t, _ in series]
        self.vs = [v for _, v in series]

    def at(self, t):
        k = bisect.bisect_right(self.ts, t) - 1
        return self.vs[k] if k >= 0 else None


def pct(values, p):
    if not values: return float('nan')
    s = sorted(values)
    return s[min(len(s) - 1, max(0, int(round(p / 100 * (len(s) - 1)))))]


def med(values):
    return statistics.median(values) if values else float('nan')


def settings_timeline(D, side):
    """[(t, (exposure_ms, gain))] when the camera's applied setting changed."""
    out = []
    exp, gain = D.get(f'RealOutputs/Vision/{side}/ExposureMs'), D.get(f'RealOutputs/Vision/{side}/Gain')
    if exp and gain:
        e, g = Track(exp), Track(gain)
        for t in sorted({t for t, _ in exp} | {t for t, _ in gain}):
            s = (e.at(t), g.at(t))
            if None not in s and s[0] >= 0 and s[1] >= 0 and (not out or out[-1][1] != s):
                out.append((t, s))
    if not out:  # Older logs: only the status text has it
        for t, v in D.get(f'RealOutputs/Vision/{side}/Status', []):
            m = re.search(r'exposure (\d+) ms, gain (\d+)', str(v))
            if m and (not out or out[-1][1] != (int(m.group(1)), int(m.group(2)))):
                out.append((t, (int(m.group(1)), int(m.group(2)))))
    return out


def speeds(D):
    pose = D.get('RealOutputs/Odometry/Pose', [])
    ts, sp = [], []
    for (t0, a), (t1, b) in zip(pose, pose[1:]):
        dt = t1 - t0
        if dt > 0 and isinstance(a, list) and isinstance(b, list) and len(a) >= 2 and len(b) >= 2:
            ts.append(t1); sp.append(math.hypot(b[0] - a[0], b[1] - a[1]) / dt)
    return Track(list(zip(ts, sp))) if ts else None


def report_camera(D, side, speed):
    p = f'Vision/{side}/'
    timeline = settings_timeline(D, side)
    end = max((t for s in D.values() for t, _ in s[-1:]), default=0)
    if not timeline:
        first = min((t for t, _ in D.get(p + 'Fps', [])), default=0)
        timeline = [(first, ('?', '?'))]
    frames = D.get(p + 'FrameNanos', [])
    get = {k: Track(D.get(p + k, [])) for k in ('TagIds', 'XIn', 'YIn', 'ZIn', 'DecisionMargin')}
    fps = D.get(p + 'Fps', [])
    total = D.get(f'RealOutputs/{p}FramesTotal', [])
    bright = {k: D.get(f'RealOutputs/{p}Brightness/{k}', []) for k in ('Mean', 'ClippedPct', 'DarkPct')}
    status = [v for _, v in D.get(f'RealOutputs/{p}Status', [])]
    restarts = [v for _, v in D.get(f'RealOutputs/{p}Restarts', [])]
    print(f'\n=== Camera {side}  (last status: {status[-1] if status else "?"}; restarts {restarts[-1] if restarts else "?"})')
    results = []
    for j, (t0, setting) in enumerate(timeline):
        t1 = timeline[j + 1][0] if j + 1 < len(timeline) else end
        a = t0 + SETTLE_S
        if t1 - a < 1:
            continue
        def in_window(s):
            # Logged on change only: the value already standing at the window's start counts too
            before = [v for t, v in s if t < a][-1:]
            vals = before + [v for t, v in s if a <= t < t1]
            return [v for v in vals if isinstance(v, (int, float)) and not math.isnan(v)]
        fps_vals = [v for v in in_window(fps) if v > 0]
        tot = in_window(total)
        n_frames = sum(max(0, y - x) for x, y in zip(tot, tot[1:])) if len(tot) > 1 else None
        if n_frames is None and fps_vals:
            n_frames = int(med(fps_vals) * (t1 - a))
        tag_frames, margins = 0, []  # margins: (margin, range m, speed)
        for t, fn in frames:
            if not (a <= t < t1) or not fn:
                continue
            ids = get['TagIds'].at(t) or []
            X, Y, Z, M = (get[k].at(t) or [] for k in ('XIn', 'YIn', 'ZIn', 'DecisionMargin'))
            got = False
            for k, tid in enumerate(ids):
                if tid < 0 or k >= len(M) or k >= len(X) or k >= len(Y) or k >= len(Z):
                    continue
                if any(isinstance(v, float) and math.isnan(v) for v in (M[k], X[k], Y[k], Z[k])):
                    continue
                got = True
                r = math.sqrt(X[k] ** 2 + Y[k] ** 2 + Z[k] ** 2) * 0.0254
                margins.append((M[k], r, speed.at(t) if speed else None))
            tag_frames += got
        dur = t1 - a
        print(f'\n  exposure {setting[0]} ms, gain {setting[1]}: {dur:.0f} s, fps median {med(fps_vals):.1f}'
              + (f', {n_frames} frames' if n_frames else ''))
        b = {k: in_window(v) for k, v in bright.items()}
        if b['Mean']:
            print(f'    brightness: mean grey {med(b["Mean"]):.0f}/255 (p90 {pct(b["Mean"], 90):.0f}), '
                  f'clipped white {med(b["ClippedPct"]):.1f}% (p90 {pct(b["ClippedPct"], 90):.1f}%), '
                  f'crushed black {med(b["DarkPct"]):.1f}% (p90 {pct(b["DarkPct"], 90):.1f}%)')
        else:
            print('    brightness: not in this log (logged from the 15 Nov exposure-logging build on)')
        rate = f'{tag_frames / n_frames:.0%} of frames' if n_frames else f'{tag_frames / dur:.1f}/s'
        ms = [m for m, _, _ in margins]
        print(f'    tags: {len(margins)} sightings in {tag_frames} frames ({rate}); margin median {med(ms):.0f}, '
              f'p10 {pct(ms, 10):.0f}; below {MIN_DECISION_MARGIN} (dropped): '
              f'{sum(m < MIN_DECISION_MARGIN for m in ms)}')
        for lo, hi in RANGE_BINS_M:
            sel = [m for m, r, _ in margins if lo <= r < hi]
            if sel:
                print(f'      {lo:.1f}-{hi if hi < 99 else "+":>3} m: n {len(sel):4d}  margin median {med(sel):5.0f}  p10 {pct(sel, 10):5.0f}')
        if speed:
            for lo, hi, label in SPEED_BINS:
                sel = [m for m, r, s in margins if s is not None and lo <= s < hi and r < 2.5]
                if sel:
                    print(f'      {label:>17} (<2.5 m): n {len(sel):4d}  margin median {med(sel):5.0f}')
        near = [m for m, r, _ in margins if r < 1.5]
        results.append(dict(setting=setting, mean=med(b['Mean']) if b['Mean'] else None,
                            clip=med(b['ClippedPct']) if b['ClippedPct'] else None,
                            clip90=pct(b['ClippedPct'], 90) if b['ClippedPct'] else None,
                            dark=med(b['DarkPct']) if b['DarkPct'] else None,
                            near=med(near) if len(near) >= 10 else None, all=med(ms) if ms else None,
                            moving=[m for m, r, s in margins if s is not None and s >= 0.3 and r < 2.5],
                            still=[m for m, r, s in margins if s is not None and s < 0.3 and r < 2.5]))
    return results


def advice(side, res):
    res = [r for r in res if r['all'] is not None]
    if not res:
        return [f'{side}: no tags seen, nothing to judge (check the camera status above)']
    r = max(res, key=lambda x: x['all'])
    exp, gain = r['setting']
    out = []
    near = f"near-range margin {r['near']:.0f} vs ~{HOME_NEAR_MARGIN} at home" if r['near'] else 'too few near tags to compare with home'
    if r['mean'] is None:
        out.append(f'{side} at {exp} ms / gain {gain}: {near}. No brightness in this log, so no direction from it.')
        return out
    blur = None
    if len(r['moving']) >= 20 and len(r['still']) >= 20:
        blur = med(r['moving']) / med(r['still'])
    bright = r['mean'] > 150 or (r['clip'] or 0) > 5 or (r['clip90'] or 0) > 15
    dark = r['mean'] < 60 or (r['dark'] or 0) > 35
    if bright:
        what = 'too bright'
        step = f'lower gain to {max(0, round(gain * 2 / 3))}' if gain > 15 else f'lower exposure to {max(1, exp - 1)} ms'
    elif dark and (r['near'] is None or r['near'] < 0.8 * HOME_NEAR_MARGIN):
        what = 'too dark'
        step = f'raise gain to {round(gain * 1.5)}' if gain < 60 else f'raise exposure to {exp + 1} ms'
    else:
        what = 'brightness fine'
        step = 'keep it'
    fmt = lambda v: 'n/a' if v is None else f'{v:.1f}%'
    out.append(f'{side} at {exp} ms / gain {gain}: mean {r["mean"]:.0f}, clipped {fmt(r["clip"])}, '
               f'dark {fmt(r["dark"])}, {near} -> {what}: {step}.')
    if blur is not None and blur < 0.75:
        out.append(f'  {side}: margins moving are {blur:.0%} of standing still: motion blur. Prefer shorter exposure '
                   f'with more gain over longer exposure.')
    return out


def main(paths):
    for path in paths:
        D = parse(path)
        op = [v for _, v in D.get('RealMetadata/OpMode Name', D.get('RealMetadata/OpMode', []))]
        light = [v for _, v in D.get('RealOutputs/Vision/Left/Lighting', [])]
        print(f'##### {path}\nOpMode {op[0] if op else "?"}; field lighting preset {light[-1] if light else "? (older log)"}')
        speed = speeds(D)
        lines = []
        for side in ('Left', 'Right'):
            lines += advice(side, report_camera(D, side, speed))
        reasons = collections.Counter(v for _, v in D.get('RealOutputs/Vision/Correction/Reason', []))
        if reasons:
            print('\n  pose correction outcomes (on change):', dict(reasons.most_common()))
        print('\n  SUGGESTION (both cameras share COMP_EXPOSURE_MS / COMP_GAIN; go with the weaker camera):')
        for line in lines:
            print('   ', line)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
