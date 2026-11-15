#!/usr/bin/env python3
"""dual.py LOG.rlog [OUT.mp4] [--start S] [--dur S] [--offset S] [--fps N] [--robotcam DIR] [--no-webcam]

One video per run, for checking and for showing: the field webcam, the robot camera (turned the right way up) and a
top-down field map side by side, with what the robot was thinking written underneath (shot state, distance,
speeds, heading error against its tolerance, flywheel and hood against target, feeder) and scrolling charts.

Log time + offset = wall time. The offset comes from LOG.clock (written when the log is pulled) or --offset.
Robot camera frames are JPEGs named by capture time in ms of the log's clock, pulled into robotcam/ (or --robotcam).
--start/--dur pick a part of the log in log seconds; the default is the whole log. Default output:
/var/mnt/timelapse/dual/<wall time>_<OpMode>.mp4"""
import sys, os, glob, math, bisect, datetime, subprocess
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rlogparse import parse

W, H = 1920, 1080
BG = (0x1e, 0x1b, 0x1a)  # BGR of #1a1b1e
PANEL = (0x28, 0x25, 0x23)
GRID = (0x3c, 0x38, 0x35)
YELLOW = (0x00, 0xF2, 0xFF)  # #FFF200
RED = (0x24, 0x1C, 0xED)  # #ED1C24
BLUE = (0xFF, 0x8C, 0x3A)
WHITE = (0xF0, 0xF0, 0xF0)
GREY = (0x9a, 0x96, 0x92)
HEADER_H = 64
CAM = (0, HEADER_H, 1248, 702)          # x, y, w, h: field webcam
DATA = (0, HEADER_H + 702, 1248, H - HEADER_H - 702)
ROBOT = (1248, HEADER_H, 672, 378)       # robot camera
MAP = (1248, HEADER_H + 378, 672, H - HEADER_H - 378)
FIELD_M = 3.658
HIVE_HALF = (0.75, 0.95)
ROBOT_M = 0.457
R = 'RealOutputs/'


def font(name, size):
    p = os.path.join(HERE, 'assets', name)
    try:
        return ImageFont.truetype(p, size)
    except OSError:
        return ImageFont.truetype('DejaVuSans.ttf', size)


F_LABEL = font('IBMPlexSans-SemiBold.ttf', 17)
F_TEXT = font('IBMPlexSans-Regular.ttf', 21)
F_VALUE = font('IBMPlexMono-SemiBold.ttf', 23)
F_STATE = font('IBMPlexSans-Bold.ttf', 40)
F_HEAD = font('IBMPlexSans-SemiBold.ttf', 24)
F_CLOCK = font('IBMPlexMono-SemiBold.ttf', 26)


def rgb(bgr):
    return (bgr[2], bgr[1], bgr[0])


class Series:
    def __init__(self, d, key):
        s = d.get(R + key, [])
        self.t = [a for a, _ in s]
        self.v = [b for _, b in s]

    def at(self, t, default=None):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else default

    def window(self, a, b):
        i, j = bisect.bisect_left(self.t, a), bisect.bisect_right(self.t, b)
        return self.t[i:j], self.v[i:j]

    def __bool__(self):
        return bool(self.t)


def spans(series, value):
    out, cur = [], None
    for t, v in zip(series.t, series.v):
        if v == value and cur is None:
            cur = t
        elif v != value and cur is not None:
            out.append((cur, t))
            cur = None
    if cur is not None:
        out.append((cur, series.t[-1]))
    return out


def num(v, fmt):
    try:
        if v is None or (isinstance(v, float) and math.isnan(v)):
            return '-'
        return fmt % v
    except TypeError:
        return str(v)


def fit(img, w, h):
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    nw, nh = max(1, int(iw * s)), max(1, int(ih * s))
    out = np.zeros((h, w, 3), np.uint8)
    out[:] = PANEL
    r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    y0, x0 = (h - nh) // 2, (w - nw) // 2
    out[y0:y0 + nh, x0:x0 + nw] = r
    return out


class Robotcam:
    def __init__(self, dirs, a, b):
        self.frames = []
        for d in dirs:
            for f in glob.glob(os.path.join(d, '*.jpg')):
                try:
                    t = int(os.path.basename(f)[:-4]) / 1000.0
                except ValueError:
                    continue
                if a - 1 <= t <= b + 1:
                    self.frames.append((t, f))
        self.frames.sort()
        self.ts = [t for t, _ in self.frames]
        self.cache = (None, None)

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        if i < 0 or t - self.ts[i] > 0.6:
            return None, None
        f = self.frames[i][1]
        if self.cache[0] != f:
            img = cv2.imread(f)
            self.cache = (f, cv2.rotate(img, cv2.ROTATE_180) if img is not None else None)
        return self.cache[1], t - self.ts[i]


class Webcam:
    """Field webcam frames in 10 s chunks, so a long run never sits in memory."""

    def __init__(self, crop, fps):
        self.crop, self.fps = crop, fps
        self.chunk = (None, None, [], None)

    def at(self, wall):
        a, b, times, fr = self.chunk
        if a is None or not (a <= wall < b):
            a = wall
            b = wall + 10.0
            try:
                import replay
                fr, times = replay.frames(a - 0.2, 10.4, self.crop, fps=self.fps)
            except SystemExit:
                fr, times = None, []
            except Exception as e:
                print('webcam:', e, file=sys.stderr)
                fr, times = None, []
            self.chunk = (a, b, times, fr)
        if not times:
            return None
        i = bisect.bisect_left(times, wall)
        if i == len(times) or (i > 0 and wall - times[i - 1] < times[i] - wall):
            i -= 1
        return fr[i] if abs(times[i] - wall) < 0.2 else None


class Map:
    def __init__(self, d, cells):
        self.x0, self.y0, self.w, self.h = MAP
        self.s = (min(self.w, self.h - 40) - 30) / FIELD_M
        self.cx, self.cy = self.w / 2, 40 + (self.h - 40) / 2
        base = np.zeros((self.h, self.w, 3), np.uint8)
        base[:] = PANEL
        half = FIELD_M / 2
        for k in range(7):
            v = -half + k * FIELD_M / 6
            cv2.line(base, self.p(v, -half), self.p(v, half), GRID, 1, cv2.LINE_AA)
            cv2.line(base, self.p(-half, v), self.p(half, v), GRID, 1, cv2.LINE_AA)
        cv2.rectangle(base, self.p(-half, half), self.p(half, -half), GREY, 2, cv2.LINE_AA)
        hx, hy = HIVE_HALF
        cv2.rectangle(base, self.p(-hx, hy), self.p(hx, -hy), (0x60, 0x5a, 0x55), 2, cv2.LINE_AA)
        for (x, y), alliance in cells.items():
            cv2.circle(base, self.p(x, y), 9, RED if alliance == 'RED' else BLUE, -1, cv2.LINE_AA)
        self.base = base

    def p(self, x, y):
        return (int(round(self.cx + x * self.s)), int(round(self.cy - y * self.s)))

    def draw(self, pose, trail, feeds, aim, lock, alliance):
        img = self.base.copy()
        for x, y in feeds:
            cv2.circle(img, self.p(x, y), 5, YELLOW, -1, cv2.LINE_AA)
        if len(trail) > 1:
            pts = np.array([self.p(x, y) for x, y in trail], np.int32)
            cv2.polylines(img, [pts], False, (0x80, 0x7a, 0x75), 2, cv2.LINE_AA)
        if pose is None:
            return img
        x, y, hd = pose
        colour = RED if alliance == 'RED' else BLUE
        if aim is not None:
            cv2.line(img, self.p(x, y), self.p(*aim), YELLOW, 1, cv2.LINE_AA)
        c, s, r = math.cos(hd), math.sin(hd), ROBOT_M / 2
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((r, r), (-r, r), (-r, -r), (r, -r))]
        cv2.polylines(img, [np.array([self.p(*q) for q in corners], np.int32)], True, colour, 3, cv2.LINE_AA)
        if lock is not None:
            cv2.line(img, self.p(x, y), self.p(x + 0.55 * math.cos(lock), y + 0.55 * math.sin(lock)),
                     WHITE, 1, cv2.LINE_AA)
        cv2.arrowedLine(img, self.p(x, y), self.p(x + 0.45 * c, y + 0.45 * s), YELLOW, 3, cv2.LINE_AA, tipLength=0.3)
        return img


def chart(img, rect, t, a_s, series_list, lo, hi, shade=(), band=None, label=''):
    """Scrolling chart of the last a_s seconds ending at t."""
    x, y, w, h = rect
    cv2.rectangle(img, (x, y), (x + w, y + h), PANEL, -1)
    sx = lambda tt: int(x + (tt - (t - a_s)) / a_s * w)
    sy = lambda v: int(y + h - (min(max(v, lo), hi) - lo) / (hi - lo) * h)
    for a, b in shade:
        if b > t - a_s and a < t:
            ov = img.copy()
            cv2.rectangle(ov, (max(x, sx(a)), y), (min(x + w, sx(min(b, t))), y + h), RED, -1)
            cv2.addWeighted(ov, 0.35, img, 0.65, 0, img)
    if band is not None:
        b0, b1 = band
        ov = img.copy()
        cv2.rectangle(ov, (x, sy(b1)), (x + w, sy(b0)), (0x50, 0x70, 0x40), -1)
        cv2.addWeighted(ov, 0.35, img, 0.65, 0, img)
    for ts, vs, colour, thick in series_list:
        pts = [(sx(tt), sy(v)) for tt, v in zip(ts, vs) if isinstance(v, (int, float)) and not math.isnan(v)]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, colour, thick, cv2.LINE_AA)


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    path = args[0]
    opt = lambda k, d=None: args[args.index(k) + 1] if k in args else d
    out = args[1] if len(args) > 1 and not args[1].startswith('--') else None
    fps = float(opt('--fps', 30))
    off = opt('--offset')
    if off is None and os.path.exists(path + '.clock'):
        off = open(path + '.clock').read().split()[0]
    off = float(off) if off is not None else None
    d, _ = parse(path)
    S = lambda k: Series(d, k)
    pose_s, state, reason = S('Odometry/Pose'), S('SOTM/State'), S('SOTM/Reason')
    feeder, gate, ball_left = S('Feeder/State'), S('Feeder/Gate'), S('Feeder/BallLeft')
    rpm, trpm, hood, thood = S('Shooter/Rpm'), S('Shooter/TargetRpm'), S('Hood/AngleDeg'), S('Hood/TargetAngleDeg')
    lip, toward, side = S('SOTM/DistanceToLipM'), S('SOTM/SpeedTowardCellMps'), S('SOTM/SidewaysSpeedMps')
    lock, tol, knows = S('SOTM/LockHeadingRad'), S('SOTM/HeadingToleranceRad'), S('SOTM/KnowsCell')
    cellx, celly, alliance = S('SOTM/CellX'), S('SOTM/CellY'), S('SOTM/Alliance')
    batt, tags, phase = S('Robot/BatteryVolts'), S('Vision/Correction/TagsUsed'), S('FieldTest/Phase')
    trial, fa = S('FieldTest/Trial'), S('FieldTest/Alliance')
    alltimes = [ts for k, v in d.items() if k.startswith(R) for ts in (v[0][0], v[-1][0]) if v]
    t_first, t_last = min(alltimes), max(alltimes)
    a = float(opt('--start', t_first))
    b = min(t_last, a + float(opt('--dur', t_last - a)))
    opmode = os.path.basename(path).split('_log_')[0]
    wall0 = (a + off) if off is not None else None
    if out is None:
        os.makedirs('/var/mnt/timelapse/dual', exist_ok=True)
        stamp = datetime.datetime.fromtimestamp(wall0).strftime('%Y%m%d_%H%M%S') if wall0 else os.path.basename(path)[:-5]
        out = '/var/mnt/timelapse/dual/%s_%s.mp4' % (stamp, opmode)
    rc_dirs = [opt('--robotcam')] if opt('--robotcam') else sorted(glob.glob(os.path.join(HERE, 'robotcam', '*')))
    robotcam = Robotcam(rc_dirs, a, b)
    webcam = Webcam(None, fps) if (off is not None and '--no-webcam' not in args) else None
    if webcam is not None:
        try:
            import replay
            webcam.crop = replay.resolve_crop('field', wall0)
        except Exception as e:
            print('no field crop (%s); whole frame' % e, file=sys.stderr)
    cells = {}
    for t, x in zip(cellx.t, cellx.v):
        y = celly.at(t)
        if isinstance(x, float) and isinstance(y, float) and not (math.isnan(x) or math.isnan(y)) and knows.at(t):
            cells.setdefault((round(x, 1), round(y, 1)), alliance.at(t))
    fmap = Map(d, cells)
    feed_spans = spans(feeder, 'FEED')
    feed_pos = []
    for s0, _ in feed_spans:
        p = pose_s.at(s0)
        feed_pos.append((s0, (p[0], p[1]) if p else None))

    # Static layer
    base = np.zeros((H, W, 3), np.uint8)
    base[:] = BG
    pil = Image.fromarray(cv2.cvtColor(base, cv2.COLOR_BGR2RGB))
    dr = ImageDraw.Draw(pil)
    lockup = os.path.join(HERE, 'assets', 'lockup_onblack.png')
    if os.path.exists(lockup):
        lg = Image.open(lockup).convert('RGBA')
        lg = lg.resize((int(lg.width * 44 / lg.height), 44))
        pil.paste(lg, (18, 10), lg)
        lx = 18 + lg.width + 18
    else:
        lx = 18
    dr.text((lx, 18), 'FTC 29550  ·  BioBuzz', font=F_HEAD, fill=rgb(WHITE))
    title = opmode
    if trial:
        title += '  ·  %s trial %s' % (fa.at(a + 1e-3) or fa.v[0], trial.v[-1])
    dr.text((560, 18), title, font=F_HEAD, fill=rgb(YELLOW))
    for (x, y, w, h), name in ((CAM, 'FIELD WEBCAM'), (ROBOT, 'ROBOT CAMERA'), (MAP, 'FIELD  ·  ODOMETRY + TAGS')):
        dr.rectangle((x + 2, y + 2, x + w - 3, y + h - 3), outline=rgb(GRID), width=1)
    base = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

    enc = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '%dx%d' % (W, H),
                            '-r', '%g' % fps, '-i', '-', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '22',
                            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out], stdin=subprocess.PIPE)
    n = int((b - a) * fps)
    print('%s: %.1f s, %d frames, robot camera %d frames, webcam %s -> %s' % (
        os.path.basename(path), b - a, n, len(robotcam.frames), 'on' if webcam else 'off', out), flush=True)
    for k in range(n):
        t = a + k / fps
        img = base.copy()
        # Webcam
        x, y, w, h = CAM
        fr = webcam.at(t + off) if webcam else None
        img[y + 34:y + h, x:x + w] = fit(fr, w, h - 34) if fr is not None else PANEL
        # Robot camera
        x, y, w, h = ROBOT
        rf, age = robotcam.at(t)
        img[y + 34:y + h, x:x + w] = fit(rf, w, h - 34) if rf is not None else PANEL
        # Map
        p = pose_s.at(t)
        tr_t, tr_v = pose_s.window(t - 6, t)
        trail = [(v[0], v[1]) for v in tr_v[::3] if v]
        feeds = [q for s0, q in feed_pos if s0 <= t and q]
        aim = (cellx.at(t), celly.at(t)) if knows.at(t) else None
        al = alliance.at(t) or fa.at(t) or 'RED'
        mx, my, mw, mh = MAP
        img[my:my + mh, mx:mx + mw] = fmap.draw(p, trail, feeds, aim, lock.at(t) if knows.at(t) else None, al)
        # Charts
        dx, dy, dw, dh = DATA
        t_rpm, v_rpm = rpm.window(t - 5, t)
        t_tr, v_tr = trpm.window(t - 5, t)
        tgt = trpm.at(t) or 3500
        chart(img, (dx + 560, dy + 44, dw - 584, 130), t, 5, [(t_tr, v_tr, YELLOW, 1), (t_rpm, v_rpm, WHITE, 2)],
              tgt - 700, tgt + 300, shade=feed_spans)
        lt, lv = lock.window(t - 5, t)
        he = []
        for tt, lk in zip(lt, lv):
            pp = pose_s.at(tt)
            he.append(math.degrees(math.atan2(math.sin(lk - pp[2]), math.cos(lk - pp[2]))) if pp and isinstance(lk, float) else float('nan'))
        tl = math.degrees(tol.at(t) or 0.07)
        chart(img, (dx + 560, dy + 210, dw - 584, 90), t, 5, [(lt, he, WHITE, 2)], -15, 15, shade=feed_spans, band=(-tl, tl))
        # Text
        pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        dr = ImageDraw.Draw(pil)
        wall = t + off if off is not None else None
        clock = datetime.datetime.fromtimestamp(wall).strftime('%Y-%m-%d  %H:%M:%S.%f')[:-4] if wall else ''
        dr.text((W - 470, 16), clock, font=F_CLOCK, fill=rgb(WHITE))
        dr.text((CAM[0] + 14, CAM[1] + 6), 'FIELD WEBCAM', font=F_LABEL, fill=rgb(GREY))
        if webcam is None or fr is None:
            dr.text((CAM[0] + 500, CAM[1] + 340), 'no webcam video', font=F_TEXT, fill=rgb(GREY))
        dr.text((ROBOT[0] + 14, ROBOT[1] + 6), 'ROBOT CAMERA', font=F_LABEL, fill=rgb(GREY))
        if rf is None:
            dr.text((ROBOT[0] + 220, ROBOT[1] + 190), 'no robot camera frames', font=F_TEXT, fill=rgb(GREY))
        dr.text((MAP[0] + 14, MAP[1] + 6), 'FIELD  ·  ODOMETRY + TAGS', font=F_LABEL, fill=rgb(GREY))
        if p:
            dr.text((MAP[0] + 330, MAP[1] + 6), 'x %+.2f  y %+.2f  %+.0f°' % (p[0], p[1], math.degrees(p[2])),
                    font=F_LABEL, fill=rgb(WHITE))
        st = state.at(t) or '-'
        feeding = feeder.at(t) == 'FEED'
        dr.text((dx + 24, dy + 10), st.replace('_', ' '), font=F_STATE,
                fill=rgb(YELLOW if st == 'READY' else RED if feeding else WHITE))
        rs = reason.at(t)
        if rs and st != 'READY':
            dr.text((dx + 24, dy + 58), str(rs)[:48], font=F_TEXT, fill=rgb(GREY))
        lk, pp = lock.at(t), p
        herr = math.degrees(math.atan2(math.sin(lk - pp[2]), math.cos(lk - pp[2]))) if isinstance(lk, float) and pp else None
        rows = [
            ('Distance to lip', num(lip.at(t), '%.2f m')),
            ('Toward / sideways', '%s / %s m/s' % (num(toward.at(t), '%+.2f'), num(side.at(t), '%+.2f'))),
            ('Heading error', '%s° (±%.1f)' % (num(herr, '%+.1f'), tl)),
            ('Flywheel', '%s / %s rpm' % (num(rpm.at(t), '%.0f'), num(trpm.at(t), '%.0f'))),
            ('Hood', '%s° → %s°' % (num(hood.at(t), '%.1f'), num(thood.at(t), '%.1f'))),
            ('Feeder', '%s%s' % (feeder.at(t) or '-', ('  ·  ' + (gate.at(t) or '-')) if gate else '')),
            ('Battery  ·  tags', '%s V  ·  %s' % (num(batt.at(t), '%.1f'), num(tags.at(t), '%d'))),
        ]
        if phase:
            rows.append(('Phase', str(phase.at(t) or '-')))
        for i, (k_, v_) in enumerate(rows):
            yy = dy + 96 + i * 27
            dr.text((dx + 24, yy), k_, font=F_TEXT, fill=rgb(GREY))
            dr.text((dx + 250, yy - 1), v_, font=F_VALUE, fill=rgb(WHITE))
        dr.text((dx + 560, dy + 14), 'FLYWHEEL  ·  target yellow, feeds red', font=F_LABEL, fill=rgb(GREY))
        dr.text((dx + 560, dy + 182), 'HEADING ERROR  ·  ±15°, green is the tolerance', font=F_LABEL, fill=rgb(GREY))
        balls = sum(1 for s0, _ in feed_spans if s0 <= t)
        dr.text((dx + dw - 200, dy + 14), 'feeds %d' % balls, font=F_LABEL, fill=rgb(YELLOW))
        img = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        enc.stdin.write(img.tobytes())
        if k % int(fps * 10) == 0 and k:
            print('  %.0f%%' % (100.0 * k / n), flush=True)
    enc.stdin.close()
    enc.wait()
    print(out)


if __name__ == '__main__':
    main()
