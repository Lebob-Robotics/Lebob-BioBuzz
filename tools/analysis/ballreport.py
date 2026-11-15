#!/usr/bin/env python3
"""ballreport.py LOG.rlog [--offset SECONDS] [--crop field|hive]

One run, ball by ball: every feed with what the robot knew at that moment (state, distance, speeds, heading
error against its tolerance, wheel and hood error) next to the flights the webcam saw leave during it, where each
went, and a picture per flight. Writes LOG_report/report.txt, overview.jpg and flight_N.jpg.

Log time + offset = wall time. The offset is read from LOG.clock (balltest.sh writes it at the start of the run:
PC clock minus hub uptime, which is the log's clock), or given with --offset. It only holds for the hub boot
that wrote the log."""
import sys, os, math, bisect, datetime
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rlogparse import parse
import replay, balltrack

R = 'RealOutputs/'


class Log:
    def __init__(self, path):
        self.d, _ = parse(path)
        self.cache = {}

    def series(self, k):
        if k not in self.cache:
            s = self.d.get(R + k, [])
            self.cache[k] = ([a for a, _ in s], [b for _, b in s])
        return self.cache[k]

    def at(self, k, t):
        ts, vs = self.series(k)
        i = bisect.bisect_right(ts, t) - 1
        return vs[i] if i >= 0 else None


def num(v, fmt='%.2f'):
    try:
        if v is None or (isinstance(v, float) and math.isnan(v)):
            return '-'
        return fmt % v
    except TypeError:
        return str(v)


def wall(t):
    d = datetime.datetime.fromtimestamp(t)
    return d.strftime('%H:%M:%S.') + '%02d' % (d.microsecond // 10000)


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    path = args[0]
    off = None
    crop = 'field'
    if '--offset' in args:
        off = float(args[args.index('--offset') + 1])
    if '--crop' in args:
        crop = args[args.index('--crop') + 1]
    if off is None and os.path.exists(path + '.clock'):
        off = float(open(path + '.clock').read().split()[0])
    if off is None:
        sys.exit('no LOG.clock next to the log: give --offset (PC epoch minus hub uptime for that boot)')
    L = Log(path)
    ts, vs = L.series('Feeder/State')
    feeds, cur = [], None
    for t, v in zip(ts, vs):
        if v == 'FEED' and cur is None:
            cur = t
        if v != 'FEED' and cur is not None:
            feeds.append((cur, t))
            cur = None
    if cur is not None:
        feeds.append((cur, ts[-1]))
    if not feeds:
        sys.exit('no feeds in the log')
    outdir = path.rsplit('.', 1)[0] + '_report'
    os.makedirs(outdir, exist_ok=True)
    a, b = feeds[0][0] - 1.0 + off, feeds[-1][1] + 2.5 + off
    fr, times = replay.frames(a, b - a, crop)
    if not len(fr):
        sys.exit('no video for %s' % wall(a))
    dets, bg = balltrack.detect(fr)
    tracks = balltrack.link(dets, times, max(30.0, fr.shape[2] / 10.0), fr.shape[1] * 0.05)
    tracks.sort(key=lambda tr: tr[0][0])
    used = set()
    lines = ['%s  offset %.2f  video %s +%.1fs  %d feeds, %d flights seen' %
             (os.path.basename(path), off, wall(a), b - a, len(feeds), len(tracks)),
             'alliance %s  trial %s' % (L.at('FieldTest/Alliance', feeds[0][0]), L.at('FieldTest/Trial', feeds[0][0])), '']
    for n, (s, e) in enumerate(feeds):
        lk, hd = L.at('SOTM/LockHeadingRad', s), L.at('Drive/HeadingRad', s)
        err = math.degrees(math.atan2(math.sin(lk - hd), math.cos(lk - hd))) if lk is not None and hd is not None else None
        tol = L.at('SOTM/HeadingToleranceRad', s)
        rpm, trpm = L.at('Shooter/Rpm', s), L.at('SOTM/TargetRpm', s)
        hood, thood = L.at('Hood/AngleDeg', s), L.at('SOTM/HoodDeg', s)
        gts, gvs = L.series('Feeder/Gate')
        seq = []
        for gt, gv in zip(gts, gvs):
            if s - 0.05 <= gt <= e + 0.5 and (not seq or seq[-1] != gv):
                seq.append(gv)
        lines.append('FEED %d  %s  %.2fs  state %s  lip %s m  toward %s sideways %s m/s  accel %s  '
                     'heading err %s deg (tol %s)  rpm %s (target %s)  hood %s (target %s)' % (
                         n + 1, wall(s + off), e - s, L.at('SOTM/State', s), num(L.at('SOTM/DistanceToLipM', s)),
                         num(L.at('SOTM/SpeedTowardCellMps', s)), num(L.at('SOTM/SidewaysSpeedMps', s)),
                         num(L.at('SOTM/AccelMps2', s)), num(err, '%+.1f'),
                         num(math.degrees(tol) if tol is not None else None, '%.1f'),
                         num(rpm, '%.0f'), num(trpm, '%.0f'), num(hood, '%.1f'), num(thood, '%.1f')))
        if seq:
            lines.append('    gate ' + ' > '.join(seq))
        for k, tr in enumerate(tracks):
            t0 = tr[0][0] - off
            if k not in used and s - 0.05 <= t0 <= e + 0.45:
                used.add(k)
                lines.append('    flight %d: seen from +%.2f s after feed start: %s' % (
                    k + 1, t0 - s, balltrack.describe(tr, balltrack.cells_in_crop(a, replay.resolve_crop(crop, a), fr.shape[1:3]))))
    for k, tr in enumerate(tracks):
        if k not in used:
            lines.append('flight %d outside any feed: %s' % (k + 1, wall(tr[0][0])))
    # Balls leaving close together: the second left before the flywheel came back
    gaps = [tracks[k + 1][0][0] - tracks[k][0][0] for k in range(len(tracks) - 1)]
    close = [(k + 1, k + 2, g) for k, g in enumerate(gaps) if g < 0.3]
    lines.append('')
    lines.append('flights seen %d of %d feeds; gaps between flights %s s' % (
        len(tracks), len(feeds), ', '.join('%.2f' % g for g in gaps) or '-'))
    for a_, b_, g in close:
        lines.append('TOO CLOSE: flights %d and %d left %.2f s apart (double feed)' % (a_, b_, g))
    # Pictures: the whole window, then each flight close up
    img = cv2.addWeighted(bg, 0.75, np.full_like(bg, 255), 0.25, 0)
    for k, tr in enumerate(tracks):
        for j in range(1, len(tr)):
            cv2.line(img, (int(tr[j - 1][1]), int(tr[j - 1][2])), (int(tr[j][1]), int(tr[j][2])), (0, 0, 255), 2)
        cv2.putText(img, str(k + 1), (int(tr[0][1]) + 6, int(tr[0][2])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.imwrite(os.path.join(outdir, 'overview.jpg'), img)
    for k, tr in enumerate(tracks):
        t0 = tr[0][0]
        out = os.path.join(outdir, 'flight_%d.jpg' % (k + 1))
        os.system('%s %s %.3f 1.8 %s hive > /dev/null 2>&1' % (sys.executable, os.path.join(HERE, 'balltrack.py'),
                                                               t0 - 0.3, out))
    txt = '\n'.join(lines)
    open(os.path.join(outdir, 'report.txt'), 'w').write(txt + '\n')
    print(txt)
    print(outdir)


if __name__ == '__main__':
    main()
