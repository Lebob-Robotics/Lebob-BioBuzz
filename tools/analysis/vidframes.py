#!/usr/bin/env python3
"""vidframes.py: frames from the field webcam by wall-clock time, exact to the frame.

record.sh stamps every frame with its capture time (-ts mono2abs -copyts), stored modulo the MPEG-TS wrap
(2^33 / 90 kHz), so a frame's wall time is its PTS plus a whole number of wraps. Seeking with -ss before -i jumps
forward to the next keyframe (one a second), so the decode starts a keyframe early and frames are picked by their
own timestamps.

  frames(w0, w1, crop=None, fps=None) -> (list of BGR uint8 arrays, list of wall times)
    crop: (x, y, w, h) in full-picture pixels. fps: keep about this many frames a second (default all, 30)."""
import os, glob, datetime, subprocess, re
import numpy as np

VIDEO = '/var/mnt/timelapse/video'
WRAP = 2 ** 33 / 90000.0
SIZE = (1920, 1080)


def _segments(w0, w1):
    segs = sorted(glob.glob(os.path.join(VIDEO, '*.ts')))
    starts = [datetime.datetime.strptime(os.path.basename(s)[:15], '%Y%m%d_%H%M%S').timestamp() for s in segs]
    out = []
    for i, (s, t) in enumerate(zip(segs, starts)):
        end = starts[i + 1] if i + 1 < len(segs) else 1e18
        if t - 2 < w1 and end + 2 > w0:
            out.append((s, t))
    return out


def _decode(path, name_t, w0, w1, crop):
    x, y, w, h = crop or (0, 0) + SIZE
    seek = max(0.0, w0 - name_t - 1.5)
    # -t before -i: as an output option it counts from 0 and, with -copyts, ends the output at once
    p = subprocess.run(['ffmpeg', '-v', 'info', '-hide_banner', '-nostats', '-ss', '%.3f' % seek,
                        '-t', '%.3f' % (w1 - w0 + 3.0), '-i', path, '-copyts', '-vf', 'showinfo,crop=%d:%d:%d:%d' % (w, h, x, y),
                        # One frame out per frame decoded: a constant-rate output pads with copies (the camera
                        # really gives about 25 a second) and frame i no longer matches the i-th timestamp
                        '-fps_mode', 'passthrough', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'], capture_output=True)
    pts = [float(m) for m in re.findall(r'Parsed_showinfo.* pts_time:([0-9.]+)', p.stderr.decode(errors='replace'))]
    n = min(len(p.stdout) // (w * h * 3), len(pts))
    arr = np.frombuffer(p.stdout[:n * w * h * 3], np.uint8).reshape(n, h, w, 3)
    walls = [q + round((name_t - q) / WRAP) * WRAP for q in pts[:n]]
    return [(t, arr[i]) for i, t in enumerate(walls) if w0 <= t <= w1]


def frames(w0, w1, crop=None, fps=None):
    got = []
    for path, t in _segments(w0, w1):
        got += _decode(path, t, w0, w1, crop)
    got.sort(key=lambda q: q[0])
    keep, last = [], -1e9
    for t, f in got:
        if keep and t - keep[-1][0] < 1e-3:
            continue
        if fps and t - last < 1.0 / fps - 0.01:
            continue
        keep.append((t, f))
        last = t
    return [f for _, f in keep], [t for t, _ in keep]
