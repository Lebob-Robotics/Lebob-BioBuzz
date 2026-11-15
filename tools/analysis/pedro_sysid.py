import sys, glob, math, json
import rlogparse
from multiprocessing import Pool
IN = 0.0254

def at_series(s):
    return s if s else []

def one(path):
    try:
        S, _ = rlogparse.parse(path)
    except Exception as e:
        return None
    P = S.get('RealOutputs/Odometry/Pose'); V = S.get('RealOutputs/Odometry/VelocityField'); W = S.get('RealOutputs/Drive/WheelPowers')
    Z = S.get('HardwareMap/DriveBL/zeroPowerBehavior', [])
    F = S.get('RealOutputs/Odometry/Faulted', [])
    if not P or not V or not W: return None
    # align on pose timestamps
    def sampler(s):
        i = [0]
        def f(t):
            while i[0] + 1 < len(s) and s[i[0] + 1][0] <= t: i[0] += 1
            return s[i[0]][1] if s and s[0][0] <= t else None
        return f
    v_at, w_at, z_at, f_at = sampler(V), sampler(W), sampler(Z), sampler(F)
    rows = []
    for t, p in P:
        v = v_at(t); w = w_at(t)
        if v is None or w is None: continue
        h = p[2]; c, s = math.cos(h), math.sin(h)
        fwd = v[0] * c + v[1] * s; left = -v[0] * s + v[1] * c
        rows.append((t, p[0], p[1], fwd, left, max(abs(x) for x in w), list(w), str(z_at(t)), bool(f_at(t))))
    # max velocity: full power, mostly one axis, 10-sample mean
    full = {'fwd': [], 'str': []}
    for i in range(10, len(rows)):
        win = rows[i-10:i]
        if all(r[5] >= 0.95 for r in win) and not any(r[8] for r in win):
            mf = sum(abs(r[3]) for r in win) / 10; ml = sum(abs(r[4]) for r in win) / 10
            # pure forward: wheels all same sign and similar magnitude
            ws = win[-1][6]
            if mf > 3 * ml: full['fwd'].append(mf)
            elif ml > 3 * mf: full['str'].append(ml)
    # stops: power falls to ~0 while moving, stays 0 until stopped
    stops = []
    i = 1
    while i < len(rows):
        r0 = rows[i-1]; r = rows[i]
        if r0[5] > 0.3 and r[5] < 0.02 and not r[8]:
            sp = math.hypot(r[3], r[4])
            if sp > 0.3:
                axis = 'fwd' if abs(r[3]) > 3 * abs(r[4]) else 'str' if abs(r[4]) > 3 * abs(r[3]) else None
                j = i; dist = 0; ok = True
                while j + 1 < len(rows) and math.hypot(rows[j][3], rows[j][4]) > 0.025:
                    if rows[j+1][5] >= 0.02 or rows[j+1][8]: ok = False; break
                    dist += math.hypot(rows[j+1][1] - rows[j][1], rows[j+1][2] - rows[j][2])
                    j += 1
                dt = rows[j][0] - r[0]
                if ok and axis and dt > 0.05 and dist < 2.0:
                    v0 = abs(r[3]) if axis == 'fwd' else abs(r[4])
                    stops.append({'axis': axis, 'v0': v0, 'dist': dist, 'dt': dt, 'zpb': r[7], 'file': path.split('/')[-1]})
                i = j
        i += 1
    return full, stops

if __name__ == '__main__':
    files = sorted(glob.glob(sys.argv[2] if len(sys.argv) > 2 else 'logs/*20261113*.rlog'))
    with Pool() as pool:
        res = [r for r in pool.map(one, files) if r]
    full = {'fwd': [], 'str': []}; stops = []
    for f, s in res:
        full['fwd'] += f['fwd']; full['str'] += f['str']; stops += s
    json.dump({'full': full, 'stops': stops}, open(sys.argv[1], 'w'))
    print('files', len(res), 'full-power windows fwd', len(full['fwd']), 'str', len(full['str']), 'stops', len(stops))
