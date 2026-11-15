import sys, bisect, math, collections
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from rlogparse import parse
import numpy as np

NAMES = ['still', 'strafe full', 'back across nearer', 'strafe 0.6 far', 'diag in far corner', 'diag back out',
         'in from near corner', 'straight at', 'straight away', 'slow strafe 0.3', 'standing far',
         'corner to corner', 'corner back']
d, _ = parse(sys.argv[1])
def s(k): return d.get('RealOutputs/' + k, [])
cache = {}
def at(k, t):
    if k not in cache: cache[k] = ([x[0] for x in s(k)], s(k))
    ts, ser = cache[k]; i = bisect.bisect_right(ts, t) - 1
    return ser[i][1] if i >= 0 else None
fin = [x for x in s('FieldTest/FinishReason') if x[1]]
loops = [t for t, _ in s('Loop/StartMs')]
print('log %.0f s, finish: %s' % ((loops[-1] - loops[0]) if loops else 0, fin[0][1] if fin else '(running)'))
ph = s('FieldTest/Phase')
ch = [(t, v) for i, (t, v) in enumerate(ph) if i == 0 or ph[i - 1][1] != v]
rows = []
for i, (t0, p) in enumerate(ch):
    if p not in ('MOVE', 'PRE_AIM'): continue
    t1 = ch[i + 1][0] if i + 1 < len(ch) else loops[-1]
    lt = [t for t in loops if t0 <= t < t1]
    if not lt: continue
    tr = at('FieldTest/Trial', t0); lap = at('FieldTest/Lap', t0) or 0; al = at('FieldTest/Alliance', t0) or 'RED'
    st = [at('SOTM/State', t) for t in lt]
    ready = [x == 'READY' for x in st]
    first = next((lt[j] - t0 for j in range(len(lt)) if ready[j]), None)
    fails = collections.Counter(); reasons = collections.Counter(); hdg = []; spd = []; dist = []
    for t in lt:
        tgt = at('Shooter/TargetRpm', t) or 0; L = at('Shooter/RpmLeft', t) or 0; R = at('Shooter/RpmRight', t) or 0
        e = (L + R) / 2 - tgt
        if 'RealOutputs/SOTM/Gate/Hood' in d:
            if not at('SOTM/Gate/Flywheels', t): fails['rpm'] += 1
            if not at('SOTM/Gate/Hood', t): fails['hood'] += 1
            if at('SOTM/Valid', t) and not at('SOTM/Gate/AimReady', t): fails['aim'] += 1
        else:
            if not (abs(L - tgt) <= 125 and abs(R - tgt) <= 125 and -100 <= e <= 50): fails['rpm'] += 1
            if not at('Hood/AtTarget', t): fails['hood'] += 1
        th = at('SOTM/TargetHeadingRad', t) or 0; h = math.radians(at('Odometry/HeadingDegrees', t) or 0)
        tol = at('SOTM/HeadingToleranceRad', t) or 0.1
        he = (th - h + math.pi) % (2 * math.pi) - math.pi; hdg.append(abs(math.degrees(he)))
        if abs(he) > tol: fails['hdg'] += 1
        if not at('SOTM/Valid', t): reasons[str(at('SOTM/Reason', t))] += 1
        spd.append(math.hypot(at('SOTM/SpeedTowardCellMps', t) or 0, at('SOTM/SidewaysSpeedMps', t) or 0))
        dist.append(at('SOTM/DistanceToLipM', t) or float('nan'))
    n = len(lt)
    name = NAMES[tr] if tr is not None and tr < len(NAMES) else str(tr)
    rows.append((lap, al, tr, name, p, t1 - t0, n, first, np.mean(ready), fails, reasons, np.median(hdg), max(hdg), max(spd), np.nanmin(dist), np.nanmax(dist)))
for r in rows:
    lap, al, tr, name, p, dur, n, first, rd, fails, reasons, hmed, hmax, vmax, dmin, dmax = r
    rs = ' '.join('%s %d' % (k[:12], v) for k, v in reasons.most_common(2))
    print('L%d %-4s %2s %-20s %-7s %4.1fs first %5s READY %3.0f%% | not: rpm %2d hood %2d hdg %2d aim %2d | hdgErr med %4.1f max %5.1f | v %.2f d %.2f-%.2f | %s' % (
        lap, al[:4], tr, name[:20], p, dur, ('%.2f' % first) if first is not None else 'never', rd * 100,
        fails['rpm'], fails['hood'], fails['hdg'], fails['aim'], hmed, hmax, vmax, dmin, dmax, rs))
mv = [r for r in rows if r[4] == 'MOVE']
if mv:
    print('MOVE summary: %d trials, READY at some point in %d, median first READY %.2f s, mean READY %.0f%%' % (
        len(mv), sum(1 for r in mv if r[7] is not None),
        np.median([r[7] for r in mv if r[7] is not None]) if any(r[7] is not None for r in mv) else float('nan'),
        100 * np.mean([r[8] for r in mv])))
skips = s('FieldTest/Skipped')
print('skipped', skips[-1][1] if skips else 0)
