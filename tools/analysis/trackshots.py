#!/usr/bin/env python3
"""trackshots.py LOG.rlog [--save]

Judges a grid run's shots from the side field webcam alone (frames dated exactly by vidframes).

Between contacts a ball flies a fixed arc in the picture: x straight in time, y a parabola (gravity). So rather
than linking blobs frame to frame (two balls in the air swap), arcs are fitted to all of the run's ball blobs by
RANSAC, best first; a bounce starts a new arc where the last one ended, so chaining arcs gives each ball's path
and its contacts. A launch is an arc that starts low and climbs fast. Then:
  IN: the path ends inside the up Cell's outline (the ball stops moving there);
  OUT: it went inside and left; MISS: it never got inside;
  a path lost on the way down just above the opening counts as in (the Cell's bars hide a ball dropping in), and
  an arc no path took that leaves the Cell marks the ball that went in last as OUT (bounced out);
  plus short/long (where the descent passes the Cell) and toward/away from the camera (picture grows/shrinks).
Each launch gets the log's values at its release. Writes /var/mnt/timelapse/grid/<log>_shots.jpg; with --save
also its rows in judged_shots.csv (replacing that log's). verdict_fixes.csv overrides verdicts; DROP removes a
launch that was another ball's fragment. --calls in,in,short (in, short, long or out per ball)
skips the video and takes those verdicts, for a run the webcam missed."""
import os, sys, math, datetime, random, csv
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vidframes
from rlogparse import parse

OUT = '/var/mnt/timelapse/grid'
JUDGED = os.path.join(HERE, 'judged_shots.csv')  # One row per launch, rewritten per log with --save
FIXES = os.path.join(HERE, 'verdict_fixes.csv')  # Verdicts checked by eye on the frames, over the tracker's
S = 0.5  # Detection scale: a ball is still 10-25 px across
# The up Cell in the side view (full-picture pixels, camera placed 6 Nov 09:42): opening's top-left corner,
# top, far top corner, far bottom corner, bottom near the opening
CELL = np.array([(905, 230), (1045, 155), (1200, 280), (1100, 425), (930, 340)], np.int32)
MERGE_PX = 30  # Blobs this close are one ball split by a bar or blur
FIT_PX = 14  # A blob within this of an arc belongs to it
G_RANGE = (1200, 9000)  # Picture gravity, px/s^2 (about 3200 for a ball near the Hive)
MAX_GAP_S = 0.2  # An arc's blobs are never further apart in time (bars hide the ball for a few frames)
MIN_POINTS = 5
# The next arc of a path starts this close to where the last ended; a rebound away from the Cell up to 400 px/s of gap
# further (a ball off the top bar showed 64 px on, 0.1 s later: 6 Nov, 092634). Two balls fed together are never
# chained that way, as both fly toward the Cell
CHAIN_PX, CHAIN_S = 60, 0.25
TOWARD = 1.6  # Picture grew this much on the way down: came toward the camera
# A path lost on the way down whose arc enters the Cell within this went in: the Cell's bars hid a ball dropping
# through the opening for 0.34 s (6 Nov, 090415 ball 1), longer than CHAIN_S
LOST_S = 0.2
PAIR_S = 0.3
ESCAPE_PX = 60  # An unchained arc from inside the Cell to this far outside it is a ball bouncing out


def window(log):
    d, _ = parse(log)
    R = 'RealOutputs/'
    ph = d[R + 'FieldTest/Phase']
    a = next(t for t, v in ph if v == 'PRE_AIM')
    b = next((t for t, v in ph if v == 'DONE'), ph[-1][0])
    feeds, prev = [], None
    for t, v in d.get(R + 'Feeder/State', []):
        if v == 'FEED' and prev != 'FEED' and a <= t <= b:
            feeds.append(t)
        prev = v
    dips = [t for t, v in d.get(R + 'Feeder/BallLeft', []) if v and a <= t <= b + 1]
    return a, b, feeds, dips


def unique(fs, ts):
    """Drops the frames the webcam repeats (about one in six). A repeat differs from the frame before in at most a
    pixel or two by more than 20 levels at 480x270; a new frame in 20 or more."""
    keep_f, keep_t, last = [], [], None
    for f, t in zip(fs, ts):
        g = cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (480, 270)).astype(np.int16)
        if last is not None and (np.abs(g - last) > 20).sum() < 5:
            continue
        keep_f.append(f)
        keep_t.append(t)
        last = g
    return keep_f, keep_t


def yellow(f):
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (h >= 20) & (h <= 40) & (s >= 45) & (v >= 120)


def detect(fs, ts):
    """All moving yellow blobs as rows (t, x, y, area) in full-picture pixels: yellow pixels that differ from the
    median of frames 5-9 either side (the driving robot, people and resting balls drop out), near pieces merged."""
    n = len(fs)
    sm = [f.astype(np.int16) for f in fs]
    rows = []
    for i in range(n):
        ref = [j for j in (i - 9, i - 7, i - 5, i + 5, i + 7, i + 9) if 0 <= j < n]
        bg = np.median(np.stack([sm[j] for j in ref]), axis=0)
        m = ((np.abs(sm[i] - bg).max(axis=2) > 28) & yellow(fs[i])).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        k, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
        blobs = [[cen[c][0] / S, cen[c][1] / S, st[c, 4] / S / S] for c in range(1, k) if st[c, 4] >= 6]
        merged = True
        while merged:
            merged = False
            for p in range(len(blobs)):
                for q in range(p + 1, len(blobs)):
                    a, b = blobs[p], blobs[q]
                    if math.hypot(a[0] - b[0], a[1] - b[1]) < MERGE_PX:
                        w = a[2] + b[2]
                        blobs[p] = [(a[0] * a[2] + b[0] * b[2]) / w, (a[1] * a[2] + b[1] * b[2]) / w, w]
                        del blobs[q]
                        merged = True
                        break
                if merged:
                    break
        rows += [(ts[i], b[0], b[1], b[2]) for b in blobs]
    return np.array(rows) if rows else np.zeros((0, 4))


def fit(P):
    """Least-squares arc through rows P: x = a + b t, y = c + d t + e t^2, t from the first row."""
    t = P[:, 0] - P[0, 0]
    cx = np.polyfit(t, P[:, 1], 1)
    cy = np.polyfit(t, P[:, 2], 2)
    return P[0, 0], cx, cy


def residual(arc, D):
    t0, cx, cy = arc
    t = D[:, 0] - t0
    return np.hypot(np.polyval(cx, t) - D[:, 1], np.polyval(cy, t) - D[:, 2])


def support(arc, D, seed_t):
    """The arc's inliers, one per frame, in the contiguous run of frames (gaps up to MAX_GAP_S) around seed_t."""
    r = residual(arc, D)
    idx = np.where(r < FIT_PX)[0]
    best = {}
    for i in idx:
        t = D[i, 0]
        if t not in best or r[i] < r[best[t]]:
            best[t] = i
    ts = sorted(best)
    if not ts:
        return []
    k = min(range(len(ts)), key=lambda j: abs(ts[j] - seed_t))
    lo = hi = k
    while lo > 0 and ts[lo] - ts[lo - 1] <= MAX_GAP_S:
        lo -= 1
    while hi < len(ts) - 1 and ts[hi + 1] - ts[hi] <= MAX_GAP_S:
        hi += 1
    return [best[t] for t in ts[lo:hi + 1]]


def arcs(D, tries=1500, seed=1):
    rnd = random.Random(seed)
    left = np.ones(len(D), bool)
    found = []
    while left.sum() >= MIN_POINTS:
        idx = np.where(left)[0]
        Dl = D[idx]
        best = None
        for _ in range(tries):
            i = rnd.randrange(len(Dl))
            near = np.where((np.abs(Dl[:, 0] - Dl[i, 0]) < 0.35) & (Dl[:, 0] != Dl[i, 0]))[0]
            if len(near) < 2:
                continue
            j, k = rnd.sample(list(near), 2)
            P = Dl[[i, j, k]]
            if len(set(P[:, 0])) < 3:
                continue
            P = P[np.argsort(P[:, 0])]
            arc = fit(P)
            g = 2 * arc[2][0]
            if not (G_RANGE[0] <= g <= G_RANGE[1]):
                continue
            sup = support(arc, Dl, Dl[i, 0])
            if best is None or len(sup) > len(best[1]):
                best = (arc, sup)
        if best is None or len(best[1]) < MIN_POINTS:
            break
        # Refit on all inliers and take the support again
        P = Dl[best[1]]
        P = P[np.argsort(P[:, 0])]
        arc = fit(P)
        sup = support(arc, Dl, P[len(P) // 2, 0]) or best[1]
        if len(sup) < MIN_POINTS:
            left[idx[best[1]]] = False
            continue
        P = Dl[sup]
        P = P[np.argsort(P[:, 0])]
        found.append({'arc': fit(P), 'pts': P})
        left[idx[sup]] = False
    return found


def at(a, t):
    t0, cx, cy = a['arc']
    return np.polyval(cx, t - t0), np.polyval(cy, t - t0)


def chain(launch, rest):
    path = [launch]
    while True:
        end = path[-1]['pts'][-1]
        nxt = None
        for a in rest:
            s = a['pts'][0]
            if 0 <= s[0] - end[0] <= CHAIN_S:
                px, py = at(path[-1], s[0])
                if math.hypot(s[1] - px, s[2] - py) < CHAIN_PX + 400 * (s[0] - end[0]) or \
                        math.hypot(s[1] - end[1], s[2] - end[2]) < CHAIN_PX + 400 * (s[0] - end[0]) * (a['arc'][1][0] < 0):
                    if nxt is None or s[0] < nxt['pts'][0][0]:
                        nxt = a
        if nxt is None:
            return path
        del rest[next(i for i, a in enumerate(rest) if a is nxt)]
        path.append(nxt)


def inside(x, y, margin=0):
    return cv2.pointPolygonTest(CELL, (float(x), float(y)), True) >= -margin


def judge(path):
    P = np.vstack([a['pts'] for a in path])
    apex = P[np.argmin(P[:, 2])]
    desc = P[P[:, 0] >= apex[0]]
    ins = [inside(q[1], q[2]) for q in desc]
    grow = np.median(desc[-4:, 3]) / max(1.0, np.median(P[:6, 3]))
    side = 'toward camera' if grow >= TOWARD else ('away from camera' if grow <= 1 / TOWARD else '')
    contacts = ['(%d,%d) +%.2f' % (a['pts'][0][1], a['pts'][0][2], a['pts'][0][0] - P[0, 0]) for a in path[1:]]
    if not any(ins):
        t_end = P[-1, 0]
        falling = np.polyval(np.polyder(path[-1]['arc'][2]), t_end - path[-1]['arc'][0]) > 0
        entry = next((t for t in np.arange(t_end, t_end + LOST_S, 0.02) if inside(*at(path[-1], t))), None)
        if falling and entry is not None:
            return 'IN', 'lost entering', apex, contacts, P, entry
        mid = (CELL[:, 1].min() + CELL[:, 1].max()) / 2.0
        cross = next((q for q in desc if q[2] >= mid), desc[-1])
        where = 'short' if cross[1] < CELL[:, 0].min() + 40 else 'long'
        return 'MISS', where + (' ' + side if side else ''), apex, contacts, P, None
    k = ins.index(True)
    entry = desc[k][0]
    gone = [q for q in desc[k:] if not inside(q[1], q[2], -30)]
    if inside(desc[-1][1], desc[-1][2], 15) and not any(not inside(q[1], q[2], 30) for q in desc[k:]):
        return 'IN', '', apex, contacts, P, entry
    if gone:
        return ('OUT', 'left at (%d,%d)%s' % (gone[0][1], gone[0][2], ' ' + side if side else ''), apex, contacts, P,
                entry)
    return 'IN', '', apex, contacts, P, entry


def distance_at(path, q):
    """Pixels from blob q (t, x, y) to the path at q's time, plus 400 px per second the path had ended before it."""
    for a in path:
        if a['pts'][0][0] <= q[0] <= a['pts'][-1][0]:
            return math.hypot(*(np.array(at(a, q[0])) - q[1:3]))
    end = max((a['pts'][-1] for a in path if a['pts'][-1][0] <= q[0]), key=lambda p: p[0], default=None)
    if end is None:
        return float('inf')
    return math.hypot(end[1] - q[1], end[2] - q[2]) + 400 * (q[0] - end[0])


def one_per_ball(launches):
    """Drops a launch arc that is a piece of another's ball: starting within PAIR_S of it and never in the same
    frame, where two balls fed together show in the same frames for most of their flights. One ball's climb was
    fitted as a launch of its own peaking low (6 Nov, 091739 launch 1)."""
    keep = []
    for l in launches:
        twin = next((k for k in keep if abs(l['pts'][0][0] - k['pts'][0][0]) < PAIR_S
                     and len(set(l['pts'][:, 0]) & set(k['pts'][:, 0])) <= 1), None)
        if twin is None:
            keep.append(l)
        elif len(l['pts']) > len(twin['pts']):
            keep[keep.index(twin)] = l
    return keep


def escapes(rest):
    """Arcs no launch path took that start inside the Cell and end well outside: balls bouncing out."""
    return sorted([a for a in rest if inside(a['pts'][0][1], a['pts'][0][2])
                   and not inside(a['pts'][-1][1], a['pts'][-1][2], ESCAPE_PX)], key=lambda a: a['pts'][0][0])


def log_values(d, off, t_launch, feeds, dips):
    """The log at this ball's release: the feed it came from (the last before the launch) and its dip if one came
    within 0.8 s, else 0.1 s before the ball showed (two balls out of one feed share it)."""
    R = 'RealOutputs/'

    def at(k, t):
        v = [b for a, b in d.get(R + k, []) if a <= t]
        return v[-1] if v else None
    tl = t_launch - off
    f = max([x for x in feeds if x <= tl], default=None)
    dip = next((x for x in dips if f is not None and f <= x <= min(f + 0.8, tl + 0.1)), None)
    t = dip if dip is not None else tl - 0.1
    pose = at('Odometry/Pose', t)
    cx, cy = at('SOTM/CellX', t), at('SOTM/CellY', t)
    lock = at('SOTM/LockHeadingRad', t)
    head = at('Drive/HeadingRad', t)
    err = math.degrees((head - lock + math.pi) % (2 * math.pi) - math.pi) if None not in (head, lock) else float('nan')
    return dict(x='%.2f' % pose[0], y='%.2f' % pose[1],
                lip_m='%.3f' % at('SOTM/DistanceToLipM', t),
                off_deg='%.0f' % math.degrees(math.atan2(abs(pose[1] - cy), abs(pose[0] - cx))),
                hood='%.1f' % at('Hood/AngleDeg', t), hood_tgt='%.1f' % at('Hood/TargetAngleDeg', t),
                rpm='%.0f' % at('Shooter/Rpm', (f if f is not None else t) + 0.15),
                rpm_tgt='%.0f' % at('Shooter/TargetRpm', t), rpm_trim='%.0f' % ((at('SOTM/RpmTrim', t) or 0) + (at('SOTM/TableRpmOffset', t) or 0)),
                head_trim='%.1f' % (at('SOTM/HeadingTrimDeg', t) or 0),
                hood_trim='%.1f' % -(at('SOTM/HoodLiftTrimDeg', t) or 0),
                backward='1' if at('SOTM/Backward', t) else '0',
                head_err_deg='%+.1f' % err,
                feed_to_launch_s='%.2f' % (tl - f) if f is not None else '', dip='1' if dip is not None else '0')


FIELDS = ['log', 'launch', 'alliance', 'grid', 'wall', 'x', 'y', 'lip_m', 'off_deg', 'hood', 'hood_tgt', 'rpm',
          'rpm_tgt', 'rpm_trim', 'head_trim', 'hood_trim', 'backward', 'head_err_deg', 'feed_to_launch_s', 'dip', 'verdict', 'detail', 'apex_x', 'apex_y', 'contacts']


def save(name, rows):
    old = []
    if os.path.exists(JUDGED):
        old = [r for r in csv.DictReader(open(JUDGED)) if r['log'] != name]
    with open(JUDGED, 'w', newline='') as fh:
        w = csv.DictWriter(fh, FIELDS)
        w.writeheader()
        w.writerows(old + rows)


def from_calls(name, d, off, feeds, dips, calls, alliance, grid):
    """Rows for a run the webcam missed, from what was seen live: one call per ball (in, short, long or out),
    each ball at a feed's dip in order, else 0.35 s after a feed that had none."""
    times = []
    for f in feeds:
        dip = next((x for x in dips if f <= x <= f + 0.8), None)
        times.append((dip + 0.1 if dip is not None else f + 0.35) + off)
    rows = []
    for n, (call, t) in enumerate(zip(calls, times)):
        call = call.strip().lower()
        row = dict(log=name, launch=str(n + 1), alliance=alliance, grid=str(grid),
                   wall=datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4],
                   verdict='IN' if call == 'in' else 'OUT',
                   detail='' if call == 'in' else ('bounced out' if call == 'out' else call) + ': call, no video',
                   apex_x='', apex_y='', contacts='')
        row.update(log_values(d, off, t, feeds, dips))
        rows.append(row)
        print('ball %d %s lip %s hood %s rpm %s %s %s' % (n + 1, row['wall'], row['lip_m'], row['hood'], row['rpm'],
                                                          row['verdict'], row['detail']))
    return rows


def main():
    log = sys.argv[1]
    name = os.path.basename(log)[:-5]
    off = float(open(log + '.clock').read().split()[0])
    d, _ = parse(log)
    a, b, feeds, dips = window(log)
    R = 'RealOutputs/'
    alliance = (d.get(R + 'FieldTest/Alliance') or [(0, '')])[-1][1]
    grid = (d.get(R + 'FieldTest/OnlyGrid') or [(0, '')])[-1][1]
    if '--calls' in sys.argv:
        rows = from_calls(name, d, off, feeds, dips, sys.argv[sys.argv.index('--calls') + 1].split(','), alliance, grid)
        if '--save' in sys.argv:
            save(name, rows)
        return
    fs, ts = vidframes.frames(a + off, b + off + 1.5)
    if not ts or ts[-1] < b + off + 1.0:
        sys.exit('video not on disk yet up to the end of the run')
    fs, ts = unique(fs, ts)
    small = [cv2.resize(f, None, fx=S, fy=S, interpolation=cv2.INTER_AREA) for f in fs]
    D = detect(small, ts)
    found = arcs(D)

    # A launch climbs: rising at least 900 px/s at its start, from the lower part of the picture left of the Hive
    # (the robot's side), to an apex above the Cell's bottom (a ball bouncing on the floor peaks far lower)
    def vertex_y(x):
        e, dd, c = x['arc'][2]
        return c - dd * dd / (4 * e)
    launches = sorted([x for x in found if x['pts'][0][2] > 420 and x['pts'][0][1] < CELL[:, 0].min()
                       and x['arc'][2][1] < -900 and vertex_y(x) < CELL[:, 1].max()], key=lambda x: x['pts'][0][0])
    launches = one_per_ball(launches)
    rest = [x for x in found if not any(x is l for l in launches)]
    img = fs[-1].copy()
    cv2.polylines(img, [CELL], True, (0, 255, 255), 2)
    balls = [(l, chain(l, rest), '') for l in launches]
    # A flight no path took, from left of the Cell toward it and peaking above its bottom, is a ball whose climb went
    # unseen: from further back it starts at the picture's bottom edge and climbs 3000 px/s (6 Nov, 092516 ball 2)
    for x in sorted(rest, key=lambda x: x['pts'][0][0]):
        if any(x is y for y in rest) and x['pts'][0][1] < CELL[:, 0].min() and x['pts'][0][2] <= 420 \
                and x['arc'][1][0] > 150 and vertex_y(x) < CELL[:, 1].max():
            rest.remove(x)
            balls.append((x, chain(x, rest), 'climb unseen'))
    # A shot always climbs above the Cell's bottom; a path that never did is a piece of a climb fitted on its own,
    # beside the same ball found again in flight (6 Nov, 095640 launch 1)
    balls = [b for b in balls if min(a['pts'][:, 2].min() for a in b[1]) < CELL[:, 1].max()]
    balls.sort(key=lambda b: b[0]['pts'][0][0])
    # The same check as one_per_ball over whole paths, now that balls found in flight are in: a climb fitted on its
    # own beside its ball found again in flight (6 Nov, 095900 launches 1 and 2)
    kept = []
    for b in balls:
        tb = set(np.vstack([a['pts'] for a in b[1]])[:, 0])
        twin = next((k for k in kept if abs(b[0]['pts'][0][0] - k[0]['pts'][0][0]) < PAIR_S
                     and len(tb & set(np.vstack([a['pts'] for a in k[1]])[:, 0])) <= 1), None)
        if twin is None:
            kept.append(b)
        elif len(tb) > sum(len(a['pts']) for a in twin[1]):
            kept[kept.index(twin)] = b
    balls = kept
    rows, entries, drawn = [], [], []
    for n, (l, path, note) in enumerate(balls):
        verdict, detail, apex, contacts, P, entry = judge(path)
        detail = ', '.join(s for s in (detail, note) if s)
        entries.append(entry)
        t0 = P[0, 0]
        row = dict(log=name, launch=str(n + 1), alliance=alliance, grid=str(grid),
                   wall=datetime.datetime.fromtimestamp(t0).strftime('%H:%M:%S.%f')[:-4], verdict=verdict,
                   detail=detail, apex_x='%d' % apex[1], apex_y='%d' % apex[2], contacts='; '.join(contacts))
        row.update(log_values(d, off, t0, feeds, dips))
        rows.append(row)
        drawn.append((path, apex))
    # Each ball bouncing out is the ball in the Cell whose path is nearest where it starts. Taking the one that went
    # in last gave ball 3's bounce to ball 2, since ball 3's own path had already left (6 Nov, 091907)
    for e in escapes(rest):
        s, f = e['pts'][0], e['pts'][-1]
        cand = [n for n, t in enumerate(entries) if t is not None and t <= s[0]]
        if not cand:
            continue
        n = min(cand, key=lambda n: distance_at(drawn[n][0], s))
        if rows[n]['verdict'] != 'IN':
            continue
        rows[n]['verdict'] = 'OUT'
        rows[n]['detail'] = 'bounced out from (%d,%d) to (%d,%d) at %s' % (
            s[1], s[2], f[1], f[2], datetime.datetime.fromtimestamp(s[0]).strftime('%S.%f')[:5])
        drawn[n][0].append(e)
    if os.path.exists(FIXES):
        for fx in csv.DictReader(open(FIXES)):
            if fx['log'] == name and 0 < int(fx['launch']) <= len(rows):
                row = rows[int(fx['launch']) - 1]
                row['verdict'], row['detail'] = fx['verdict'], 'checked: ' + fx['detail']
    # DROP: a launch seen on the frames to be another's fragment, not a ball. Fix numbers are from before dropping
    keep = [i for i, r in enumerate(rows) if r['verdict'] != 'DROP']
    rows, drawn = [rows[i] for i in keep], [drawn[i] for i in keep]
    for n, row in enumerate(rows):
        row['launch'] = str(n + 1)
    for n, ((path, apex), row) in enumerate(zip(drawn, rows)):
        verdict, detail, contacts, t0 = row['verdict'], row['detail'], row['contacts'], path[0]['pts'][0][0]
        print('launch %d %s  lip %s m hood %s/%s rpm %s/%s head %s  apex (%d,%d) +%.2f s  contacts %s  %s %s' % (
            n + 1, row['wall'], row['lip_m'], row['hood'], row['hood_tgt'], row['rpm'], row['rpm_tgt'],
            row['head_err_deg'], apex[1], apex[2], apex[0] - t0, contacts or '-', verdict, detail))
        col = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (255, 0, 255), (0, 128, 255)][n % 5]
        for arc in path:
            tt = np.linspace(arc['pts'][0][0], arc['pts'][-1][0], 40)
            pts = np.array([at(arc, t) for t in tt], np.int32)
            cv2.polylines(img, [pts], False, col, 2)
            cv2.circle(img, (int(arc['pts'][0][1]), int(arc['pts'][0][2])), 6, col, 2)
        cv2.putText(img, '%d %s' % (n + 1, verdict), (int(apex[1]) - 20, int(apex[2]) - 12 - 24 * (n % 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    out = os.path.join(OUT, '%s_shots.jpg' % name)
    cv2.imwrite(out, img)
    if '--save' in sys.argv:
        save(name, rows)
    print(out, '%d launches, %d feeds, %d dips in the log%s' % (len(rows), len(feeds), len(dips),
                                                               ', saved' if '--save' in sys.argv else ''))


if __name__ == '__main__':
    main()
