#!/usr/bin/env python3
"""camera.py: where the Hive is in the field webcam picture, so crops survive the camera being moved.

  camera.py locate FRAME.jpg [OUT.jpg]     print the Hive and cell boxes, draw them into OUT.jpg
  camera.py calibrate [EPOCH|FRAME.jpg]    locate at that time (default: newest timelapse frame) and record it in
                                           camera.json as the position from then on
  camera.py scan [YYYYMMDD] [step]         locate every step-th timelapse frame of a day, record each new position
  camera.py crop NAME [EPOCH]              crop string w:h:x:y (fractions) for replay.py: hive, cells, upper, field

Finds the four Cell outlines by colour: thin, strongly saturated blue and red pentagons. Solid red/orange areas
(the wall, the sofa) are rejected because an outline fills little of its bounding box."""
import sys, os, json, glob, datetime, bisect
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
DB = os.path.join(HERE, 'camera.json')
FRAMES = '/var/mnt/timelapse/frames'


def _outlines(mask, min_area, max_fill):
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_area or w < 15 or h < 15:
            continue
        fill = a / float(w * h)
        if fill > max_fill or w > 4 * h or h > 4 * w:
            continue
        out.append([int(x), int(y), int(w), int(h), int(a)])
    return out


def _merge(boxes, gap):
    """Joins pieces of one outline split by glare or the support poles."""
    boxes = [b[:] for b in boxes]
    changed = True
    while changed:
        changed = False
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i], boxes[j]
                if (a[0] - gap < b[0] + b[2] and b[0] - gap < a[0] + a[2]
                        and a[1] - gap < b[1] + b[3] and b[1] - gap < a[1] + a[3]):
                    x0, y0 = min(a[0], b[0]), min(a[1], b[1])
                    x1, y1 = max(a[0] + a[2], b[0] + b[2]), max(a[1] + a[3], b[1] + b[3])
                    boxes[i] = [x0, y0, x1 - x0, y1 - y0, a[4] + b[4]]
                    del boxes[j]
                    changed = True
                    break
            if changed:
                break
    return boxes


def locate(img):
    H, W = img.shape[:2]
    s = W / 1920.0
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (3, 3), 0), cv2.COLOR_BGR2HSV)
    h, sat, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    blue = ((h >= 100) & (h <= 128) & (sat >= 110) & (v >= 70)).astype(np.uint8) * 255
    # The orange wall sits at hue 4-18 (median 13); the red Cells at -5..+3
    red = (((h <= 4) | (h >= 170)) & (sat >= 110) & (v >= 110)).astype(np.uint8) * 255

    def cands(m):
        out = []
        for b in _merge(_outlines(m, int(60 * s * s), 0.45), int(10 * s)):
            x, y, w, hh, a = b
            if a >= 250 * s * s and w >= 45 * s and hh >= 35 * s and a / float(w * hh) <= 0.4:
                out.append({'box': [x, y, w, hh], 'px': a})
        return sorted(out, key=lambda c: -c['px'])

    blues = cands(blue)
    if not blues:
        return None
    anchor = blues[0]
    ax, ay, aw, ah = anchor['box']
    acx, acy = ax + aw / 2.0, ay + ah / 2.0

    def near(c, kx, ky):
        x, y, w, hh = c['box']
        return (abs(x + w / 2.0 - acx) < kx * aw and abs(y + hh / 2.0 - acy) < ky * ah
                and 0.15 * anchor['px'] < c['px'] < 6 * anchor['px'])

    cells = [dict(anchor, colour='blue')] + [dict(c, colour='blue') for c in blues[1:] if near(c, 1.5, 1.5)][:1]
    cells += [dict(c, colour='red') for c in cands(red) if near(c, 3.0, 1.6)][:2]
    if not any(c['colour'] == 'red' for c in cells):
        return None
    xs = [c['box'][0] for c in cells] + [c['box'][0] + c['box'][2] for c in cells]
    ys = [c['box'][1] for c in cells] + [c['box'][1] + c['box'][3] for c in cells]
    hive = [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]
    for c in cells:
        c['upper'] = c['box'][1] + c['box'][3] / 2 < hive[1] + hive[3] * 0.45
    return {'size': [W, H], 'hive': hive, 'cells': cells}


def frac(box, size):
    W, H = size
    x, y, w, h = box
    return [round(x / W, 4), round(y / H, 4), round(w / W, 4), round(h / H, 4)]


def draw(img, loc, path):
    o = img.copy()
    if loc:
        x, y, w, h = loc['hive']
        cv2.rectangle(o, (x, y), (x + w, y + h), (0, 255, 255), 3)
        for c in loc['cells']:
            x, y, w, h = c['box']
            col = (255, 0, 0) if c['colour'] == 'blue' else (0, 0, 255)
            cv2.rectangle(o, (x, y), (x + w, y + h), col, 2)
            cv2.putText(o, ('U ' if c['upper'] else 'L ') + c['colour'], (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imwrite(path, o)


def frame_time(path):
    return datetime.datetime.strptime(os.path.basename(path)[:15], '%Y%m%d_%H%M%S').timestamp()


def frame_at(epoch):
    fs = sorted(glob.glob(FRAMES + '/*.jpg'))
    ts = [frame_time(f) for f in fs]
    i = max(0, bisect.bisect_right(ts, epoch) - 1)
    return fs[i]


def load():
    return json.load(open(DB)) if os.path.exists(DB) else {'positions': []}


def save(db):
    db['positions'].sort(key=lambda p: p['from'])
    json.dump(db, open(DB, 'w'), indent=1)


def record(loc, epoch, src, db):
    p = {'from': epoch, 'when': datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S'), 'source': src,
         'hive': frac(loc['hive'], loc['size']),
         'cells': [{'colour': c['colour'], 'upper': c['upper'], 'box': frac(c['box'], loc['size'])} for c in loc['cells']]}
    db['positions'] = [q for q in db['positions'] if abs(q['from'] - epoch) > 1]
    db['positions'].append(p)
    return p


def position(epoch):
    ps = load()['positions']
    ps = [p for p in ps if p['from'] <= epoch] or ps
    if not ps:
        sys.exit('camera.json has no positions; run camera.py calibrate')
    return max(ps, key=lambda p: p['from'])


def pad(box, left, right, up, down):
    x, y, w, h = box
    x0, y0 = max(0.0, x - w * left), max(0.0, y - h * up)
    x1, y1 = min(1.0, x + w * (1 + right)), min(1.0, y + h * (1 + down))
    return x0, y0, x1 - x0, y1 - y0


def crop(name, epoch):
    p = position(epoch)
    hv = p['hive']
    if name == 'hive':      # the Hive with room for the ball's arc and the robot in front of it
        b = pad(hv, 0.6, 0.6, 0.5, 1.6)
    elif name == 'cells':   # all four Cells, tight
        b = pad(hv, 0.15, 0.15, 0.2, 0.15)
    elif name == 'upper':   # the upper Cells: the outlines overlap in the picture, so the top of the Hive box
        b = pad([hv[0], hv[1], hv[2], hv[3] * 0.6], 0.25, 0.25, 0.4, 0.2)
    elif name == 'field':   # the whole mat around the Hive
        b = pad(hv, 1.6, 1.6, 0.3, 3.0)
    else:
        sys.exit('crop names: hive cells upper field')
    w, h, x, y = b[2], b[3], b[0], b[1]
    return '%.4f:%.4f:%.4f:%.4f' % (w, h, x, y)


def moved(a, b):
    return max(abs(a[i] - b[i]) for i in range(4)) > 0.02


# ---- Registration against a checked reference picture -------------------------------------------------------
REF_IMG = os.path.join(HERE, 'reference.jpg')


def _gray(img, width=1280):
    s = width / float(img.shape[1])
    return cv2.cvtColor(cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY), s


def register(img, ref=None):
    """Homography from the reference picture to img, from room features (walls, windows, kitchen, frames).
    Returns (H, inliers) or (None, 0)."""
    ref = cv2.imread(REF_IMG) if ref is None else ref
    g1, s1 = _gray(ref)
    g2, s2 = _gray(img)
    sift = cv2.SIFT_create(4000)
    k1, d1 = sift.detectAndCompute(g1, None)
    k2, d2 = sift.detectAndCompute(g2, None)
    if d1 is None or d2 is None:
        return None, 0
    m = cv2.BFMatcher().knnMatch(d1, d2, k=2)
    good = [a for a, b in (p for p in m if len(p) == 2) if a.distance < 0.75 * b.distance]
    if len(good) < 30:
        return None, len(good)
    p1 = np.float32([k1[g.queryIdx].pt for g in good]) / s1
    p2 = np.float32([k2[g.trainIdx].pt for g in good]) / s2
    Hm, inl = cv2.findHomography(p1, p2, cv2.RANSAC, 4.0)
    return Hm, int(inl.sum()) if inl is not None else 0


def _map_box(Hm, box):
    x, y, w, h = box
    pts = np.float32([[x, y], [x + w, y], [x + w, y + h], [x, y + h]]).reshape(-1, 1, 2)
    q = cv2.perspectiveTransform(pts, Hm).reshape(-1, 2)
    x0, y0 = q.min(axis=0)
    x1, y1 = q.max(axis=0)
    return [int(x0), int(y0), int(x1 - x0), int(y1 - y0)]


def locate_registered(img):
    """The reference Hive and Cells carried into img through the room features."""
    db = load()
    ref = db.get('reference')
    if not ref:
        return None, 0
    Hm, inl = register(img)
    if Hm is None or inl < 40:
        return None, inl
    H, W = img.shape[:2]
    cells = [{'colour': c['colour'], 'upper': c['upper'], 'box': _map_box(Hm, c['box']), 'px': 0} for c in ref['cells']]
    return {'size': [W, H], 'hive': _map_box(Hm, ref['hive']), 'cells': cells}, inl


def check_hive(img, loc):
    """The reference assumes the Hive stands where it stood then; the coloured outlines say whether it still does."""
    x, y, w, h = loc['hive']
    H, W = img.shape[:2]
    x0, y0 = max(0, int(x - w * 0.6)), max(0, int(y - h * 0.6))
    x1, y1 = min(W, int(x + w * 1.6)), min(H, int(y + h * 1.6))
    c = locate(img[y0:y1, x0:x1].copy())
    if not c:
        return 'colour check: no Cell outlines near the expected Hive (hidden, dark, or the Hive moved): look at the picture'
    cx = c['hive'][0] + x0 + c['hive'][2] / 2.0
    cy = c['hive'][1] + y0 + c['hive'][3] / 2.0
    dx, dy = (cx - (x + w / 2.0)) / w, (cy - (y + h / 2.0)) / h
    verdict = 'agrees' if abs(dx) < 0.12 and abs(dy) < 0.12 else 'DISAGREES: the Hive may have moved, check the picture'
    return 'colour check %s (outline centre off by %.0f%% / %.0f%% of the Hive box)' % (verdict, dx * 100, dy * 100)


def median_frame(start, end, n=40):
    fs = [f for f in sorted(glob.glob(FRAMES + '/*.jpg')) if start <= frame_time(f) <= end]
    if not fs:
        return None
    fs = fs[::max(1, len(fs) // n)]
    return np.median(np.stack([cv2.imread(f) for f in fs]), axis=0).astype(np.uint8)

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'locate':
        img = cv2.imread(sys.argv[2])
        loc = locate(img)
        print(json.dumps(loc))
        if len(sys.argv) > 3:
            draw(img, loc, sys.argv[3])
    elif cmd == 'calibrate':
        arg = sys.argv[2] if len(sys.argv) > 2 else None
        if arg and arg.endswith('.jpg'):
            path = arg
        else:
            path = frame_at(float(arg)) if arg else sorted(glob.glob(FRAMES + '/*.jpg'))[-1]
        img = cv2.imread(path)
        loc, inl = locate_registered(img)
        how = 'registered, %d matching room features' % inl
        if not loc:
            loc, how = locate(img), 'colour only (registration failed, %d features)' % inl
        if not loc:
            sys.exit('Hive not found in ' + path)
        print(how)
        print(check_hive(img, loc))
        db = load()
        p = record(loc, frame_time(path), os.path.basename(path), db)
        save(db)
        out = os.path.join(HERE, 'calib_%s.jpg' % os.path.basename(path)[:15])
        draw(img, loc, out)
        print(json.dumps(p['hive']), out, '<- look at it')
    elif cmd == 'scan':
        day = sys.argv[2] if len(sys.argv) > 2 else datetime.date.today().strftime('%Y%m%d')
        step = int(sys.argv[3]) if len(sys.argv) > 3 else 30
        fs = sorted(glob.glob('%s/%s_*.jpg' % (FRAMES, day)))[::step]
        db = load()
        last = None
        for f in fs:
            img = cv2.imread(f)
            if img is None:
                continue
            loc, inl = locate_registered(img)
            if not loc:
                continue
            fr = frac(loc['hive'], loc['size'])
            if last is None or moved(fr, last):
                p = record(loc, frame_time(f), os.path.basename(f), db)
                print(p['when'], p['hive'], '%d features;' % inl, check_hive(img, loc))
                last = fr
        save(db)
    elif cmd == 'crop':
        epoch = float(sys.argv[3]) if len(sys.argv) > 3 else datetime.datetime.now().timestamp()
        print(crop(sys.argv[2], epoch))
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
