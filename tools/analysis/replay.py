#!/usr/bin/env python3
"""replay.py START DURATION OUT.jpg|OUT.mp4 [fps] [crop] [cols]

Cuts the field webcam recording (/var/mnt/timelapse/video, 2-minute .ts files) at a wall-clock time.
  START   epoch seconds, HH:MM:SS[.s] (today, or the day of the newest video) or YYYYmmdd_HHMMSS[.s]
  OUT     .jpg contact sheet (default 6 fps, each tile labelled with its wall time) or .mp4 clip (labelled too)
  crop    hive | cells | upper | field (from camera.json, follows the camera position at that time)
          or w:h:x:y as fractions of the picture
  env W   tile width (default 420)

Sync: every segment's start is measured, not assumed. New recordings carry the wall-clock time of each frame
(record.sh: -use_wallclock_as_timestamps -copyts); older ones are synced from their burned-in clock: the moment
the seconds digit changes and which digit it shows. Results are cached in video/sync.json."""
import sys, os, glob, json, subprocess, datetime, tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VIDEO = '/var/mnt/timelapse/video'
CACHE = os.path.join(VIDEO, 'sync.json')
DIGITS = os.path.join(HERE, 'clock_digits.npy')
WRAP = 2 ** 33 / 90000.0
# Seconds digit of the burned-in clock in the 1280x720 recordings (fontsize 28 at x=10, y=10)
DIGIT_BOX = {720: (10, 31, 262, 277)}


def name_time(f):
    return datetime.datetime.strptime(os.path.basename(f)[:15], '%Y%m%d_%H%M%S').timestamp()


def probe(f):
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v', '-show_entries',
                          'stream=height:format=duration', '-show_entries', 'packet=pts_time', '-read_intervals',
                          '%+#40', '-of', 'json', f], capture_output=True, text=True).stdout
    j = json.loads(out)
    pts = sorted(float(p['pts_time']) for p in j.get('packets', []) if 'pts_time' in p)
    return pts[0] if pts else None, float(j['format'].get('duration', 0)), int(j['streams'][0]['height'])


def gray_frames(f, seconds, height):
    w = 300 if height == 720 else 420
    h = 44 if height == 720 else 60
    p = subprocess.run(['ffmpeg', '-loglevel', 'error', '-i', f, '-t', str(seconds), '-vf', 'crop=%d:%d:0:0' % (w, h),
                        '-f', 'rawvideo', '-pix_fmt', 'gray', '-'], capture_output=True)
    return np.frombuffer(p.stdout, np.uint8).reshape(-1, h, w).astype(np.int16)


def clock_sync(f, height, fps=15.0):
    """Wall time of the segment's first frame minus its name, from the burned-in clock. None if unreadable.
    Reads only the white text (the box is half transparent, so people passing behind it change its grey), uses
    every tick in the segment, and takes the low end: the clock is stamped when a frame is drawn, which lags its
    capture by a varying amount, and the video's own timeline is the capture timeline."""
    if height not in DIGIT_BOX or not os.path.exists(DIGITS):
        return None
    y0, y1, x0, x1 = DIGIT_BOX[height]
    fr = gray_frames(f, 130, height)
    if len(fr) < 40:
        return None
    u = fr[:, y0:y1, x0:x1] > 185
    tmpl = np.load(DIGITS) > 185
    change = (u[1:] != u[:-1]).sum(axis=(1, 2))
    ticks, last = [], -99
    for i in np.where(change > 12)[0] + 1:
        if i - last >= 10:
            ticks.append(int(i))
            last = i
    t0 = name_time(f)
    est = []
    for i in ticks:
        if i + 3 >= len(u):
            continue
        scores = [(u[i + 3] != tmpl[k]).sum() for k in range(10)]
        digit = int(np.argmin(scores))
        if sorted(scores)[1] < 1.5 * min(scores) + 5:
            continue  # ambiguous read
        t = i / fps
        guess = t0 + t - 1.5
        second = guess - ((guess - digit) % 10)
        if guess - second > 5:
            second += 10
        est.append(second - t - t0)
    if len(est) < 5:
        return None
    est = np.array(est)
    est = est[np.abs(est - np.median(est)) < 0.5]
    if len(est) < 5:
        return None
    return float(np.sort(est)[min(2, len(est) - 1)])


def sync(f, cache):
    key = os.path.basename(f)
    st = os.stat(f)
    c = cache.get(key)
    if c and c.get('size') == st.st_size:
        return c
    pts0, dur, height = probe(f)
    t0 = name_time(f)
    how, offset = None, None
    if pts0 is not None and pts0 > 2.0 and abs(((t0 - pts0) / WRAP) - round((t0 - pts0) / WRAP)) * WRAP < 30:
        k = round((t0 - pts0) / WRAP)
        offset, how = pts0 + k * WRAP - t0, 'pts'
    if offset is None:
        offset = clock_sync(f, height)
        how = 'clock' if offset is not None else 'guess'
        if offset is None:
            offset = -1.9
    c = {'offset': round(offset, 3), 'how': how, 'duration': dur, 'height': height, 'size': st.st_size}
    # A segment still being written gets measured again next time
    if time_now() - st.st_mtime > 5:
        cache[key] = c
    return c


def time_now():
    return datetime.datetime.now().timestamp()


def parse_start(s):
    try:
        v = float(s)
        if v > 1e9:
            return v
    except ValueError:
        pass
    if '_' in s:
        d, t = s.split('_')
        base = datetime.datetime.strptime(d, '%Y%m%d')
    else:
        t = s
        newest = sorted(glob.glob(VIDEO + '/*.ts'))[-1]
        base = datetime.datetime.strptime(os.path.basename(newest)[:8], '%Y%m%d')
    if ':' in t:
        hh, mm, ss = t.split(':')
    else:
        hh, mm, ss = t[:2], t[2:4], t[4:]
    return base.timestamp() + int(hh) * 3600 + int(mm) * 60 + float(ss)


def resolve_crop(c, start):
    if not c or ':' in c:
        return c
    sys.path.insert(0, HERE)
    import camera
    return camera.crop(c, start)


def cut(start, dur):
    """(concat list path, seek offset, segments with their sync) for a wall-clock window."""
    cache = json.load(open(CACHE)) if os.path.exists(CACHE) else {}
    segs = [(name_time(f), f) for f in sorted(glob.glob(VIDEO + '/*.ts'))]
    use = [(t, f) for i, (t, f) in enumerate(segs)
           if t - 5 < start + dur and (i + 1 == len(segs) or segs[i + 1][0] > start - 5)]
    if not use:
        sys.exit('no video for that time')
    syncs = [sync(f, cache) for _, f in use]
    json.dump(cache, open(CACHE, 'w'), indent=0)
    lst = tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False)
    for _, f in use:
        lst.write("file '%s'\n" % f)
    lst.close()
    return lst.name, start - (use[0][0] + syncs[0]['offset']), list(zip([f for _, f in use], syncs))


def frames(start, dur, crop=None, fps=None):
    """Decoded RGB frames (N, h, w, 3) and their wall-clock times for a window, optionally cropped."""
    lst, off, segs = cut(start, dur)
    crop = resolve_crop(crop, start)
    vf = ['setpts=PTS-STARTPTS']
    if crop:
        w, h, x, y = crop.split(':')
        vf.append('crop=trunc(iw*%s/2)*2:trunc(ih*%s/2)*2:iw*%s:ih*%s' % (w, h, x, y))
    if fps:
        vf.append('fps=%g' % fps)
    vf.append('showinfo')
    p = subprocess.run(['ffmpeg', '-loglevel', 'info', '-ss', '%.3f' % off, '-f', 'concat', '-safe', '0', '-i', lst,
                        '-t', '%.3f' % dur, '-vf', ','.join(vf), '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'],
                       capture_output=True)
    os.unlink(lst)
    info = [l for l in p.stderr.decode(errors='replace').splitlines() if 'Parsed_showinfo' in l and ' pts_time:' in l]
    times = [start + float(l.split('pts_time:')[1].split()[0]) for l in info]
    size = [l for l in p.stderr.decode(errors='replace').splitlines() if 'Parsed_showinfo' in l and ' s:' in l]
    wh = size[0].split(' s:')[1].split()[0] if size else None
    if not wh:
        return np.zeros((0, 1, 1, 3), np.uint8), []
    w, h = map(int, wh.split('x'))
    arr = np.frombuffer(p.stdout, np.uint8)
    n = len(arr) // (w * h * 3)
    return arr[:n * w * h * 3].reshape(n, h, w, 3), times[:n]


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    start, dur, out = parse_start(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    fps = float(sys.argv[4]) if len(sys.argv) > 4 else 6
    crop = resolve_crop(sys.argv[5] if len(sys.argv) > 5 else '', start)
    cols = int(sys.argv[6]) if len(sys.argv) > 6 else 6
    lst, off, segs = cut(start, dur)
    day = datetime.datetime.fromtimestamp(start)
    midnight = day.replace(hour=0, minute=0, second=0, microsecond=0).timestamp()
    label = "drawtext=text='%%{pts\\:hms\\:%.3f}':x=4:y=4:fontsize=%d:fontcolor=yellow:box=1:boxcolor=black@0.6"
    vf = []
    if crop:
        w, h, x, y = crop.split(':')
        vf.append('crop=iw*%s:ih*%s:iw*%s:ih*%s' % (w, h, x, y))
    if out.endswith('.jpg'):
        n = max(1, int(round(dur * fps)))
        rows = (n + cols - 1) // cols
        width = int(os.environ.get('W', '420'))
        vf += ['setpts=PTS-STARTPTS', 'fps=%g' % fps, 'scale=%d:-1' % width, label % (start - midnight, 18), 'tile=%dx%d' % (cols, rows)]
        args = ['-frames:v', '1', '-update', '1']
        dur_read = dur + 1.0  # The tile filter writes nothing until every slot is filled
    else:
        vf += ['setpts=PTS-STARTPTS', 'scale=-2:720', label % (start - midnight, 26)]
        args = ['-c:v', 'libx264', '-preset', 'veryfast', '-crf', '22']
        dur_read = dur
    cmd = ['ffmpeg', '-loglevel', 'error', '-y', '-ss', '%.3f' % off, '-f', 'concat', '-safe', '0', '-i', lst,
           '-t', '%.3f' % dur_read, '-vf', ','.join(vf)] + args + [out]
    r = subprocess.run(cmd)
    os.unlink(lst)
    print(out if r.returncode == 0 else 'ffmpeg failed %d' % r.returncode,
          ' '.join('%s:%s%+.2f' % (os.path.basename(f)[9:15], s['how'], s['offset']) for f, s in segs))


if __name__ == '__main__':
    main()
