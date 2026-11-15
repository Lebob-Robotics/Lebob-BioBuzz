"""Battery watch: pulls only the new bytes of the newest PsiKit log, keeps the battery samples."""
import json, os, struct, subprocess, sys
S = '192.168.43.1:5555'
D = '/tmp/battmon'
os.makedirs(D, exist_ok=True)
SP = D + '/state.json'
st = json.load(open(SP)) if os.path.exists(SP) else {}
ls = subprocess.run(['adb', '-s', S, 'shell', 'ls -1i /sdcard/FIRST/PsiKit/'], capture_output=True, text=True, timeout=20).stdout
rows = [l.split() for l in ls.replace('\r', '').splitlines() if l.strip()]
rows = [r for r in rows if len(r) == 2 and r[0].isdigit()]
if not rows:
    print(json.dumps({'error': 'no logs / hub offline'})); sys.exit()
f = max(rows, key=lambda r: int(r[0]))[1]
if st.get('file') != f:
    st = {'file': f, 'off': 0, 'names': {}, 'types': {}, 't': 0.0, 'samples': [], 'mins': [], 'started': True}
off = st['off']
new = subprocess.run(['adb', '-s', S, 'exec-out', 'tail -c +%d /sdcard/FIRST/PsiKit/%s' % (off + 1, f)], capture_output=True, timeout=60).stdout
data = new
i = 1 if off == 0 else 0  # first byte of the file is a version byte
n = len(data)
names = {int(k): v for k, v in st['names'].items()}
types = st['types']
t = st['t']
keep = i
while i < n:
    kind = data[i]; j = i + 1
    if kind == 0:
        if j + 8 > n: break
        t = struct.unpack('>d', data[j:j+8])[0]; j += 8
    elif kind == 1:
        if j + 4 > n: break
        kid, ln = struct.unpack('>hh', data[j:j+4]); j += 4
        if j + ln + 2 > n: break
        name = data[j:j+ln].decode('utf-8', 'replace'); j += ln
        (tl,) = struct.unpack('>h', data[j:j+2]); j += 2
        if j + tl > n: break
        typ = data[j:j+tl].decode('utf-8', 'replace'); j += tl
        names[kid] = name; types[name] = typ
    elif kind == 2:
        if j + 4 > n: break
        kid, ln = struct.unpack('>hH', data[j:j+4]); j += 4
        if j + ln > n: break
        name = names.get(kid)
        if name in ('RealOutputs/Robot/BatteryVolts', 'RealOutputs/Robot/BatteryMinVolts') and ln == 8:
            v = struct.unpack('>d', data[j:j+8])[0]
            (st['samples'] if name.endswith('/BatteryVolts') else st['mins']).append([t, v])
        j += ln
    else:
        print(json.dumps({'error': 'bad record kind %d at %d' % (kind, off + i)})); break
    i = j
    keep = i
st['off'] = off + keep
st['names'] = {str(k): v for k, v in names.items()}
st['types'] = types
st['t'] = t
for k in ('samples', 'mins'):
    st[k] = [s for s in st[k] if s[0] >= t - 60]
json.dump(st, open(SP, 'w'))
win = [v for tt, v in st['samples'] if tt >= t - 25]
mins = [v for tt, v in st['mins'] if tt >= t - 50] + [v for tt, v in st['samples'] if tt >= t - 50]
print(json.dumps({'file': f, 'mb': round(st['off'] / 1e6, 1), 't': round(t, 1),
                  'rest': round(max(win), 2) if win else None, 'dip': round(min(mins), 2) if mins else None,
                  'last': round(st['samples'][-1][1], 2) if st['samples'] else None}))
