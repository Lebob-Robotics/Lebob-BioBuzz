#!/usr/bin/env python3
"""gridreport.py LOG.rlog [BALLS]

Standing-grid ball test report: one line per ball fed (grid point, distance to the lip, angle off the Cell's
facing, hood and RPM against target at release, heading error, feed-to-release time), a strip of cropped webcam
frames around each release in /var/mnt/timelapse/grid/, and a row per ball in grid_shots.csv with an empty result
column to fill in (in / rattle / out / over / short / left / right). Needs LOG.clock (balltest.sh writes it).
BALLS (default 3) is how many were loaded: the feeder keeps feeding while RT is held, so later feeds are empty."""
import os, sys, math, csv, glob, datetime, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rlogparse import parse

VIDEO = '/var/mnt/timelapse/video'
OUT = '/var/mnt/timelapse/grid'
CSV = os.path.join(HERE, 'grid_shots.csv')
R = 'RealOutputs/'
NO_DIP_DELAY_S = 0.27  # Typical feed-to-release time when the RPM dip is missed
FIELDS = ['log', 'ball', 'alliance', 'grid', 'x', 'y', 'lip_m', 'off_deg', 'hood', 'hood_tgt', 'rpm', 'rpm_tgt',
          'head_err_deg', 'delay_s', 'dip', 'wall', 'result']


def wrap_deg(a):
    return math.degrees((a + math.pi) % (2 * math.pi) - math.pi)


CROP = 'crop=620:520:680:200'  # The Hive's Cells in the 1920x1080 webcam picture


def segment_for(wall):
    """The webcam segment holding this wall time joined to the next (a release near the end runs into it), and the
    offset into the first."""
    segs = sorted(glob.glob(os.path.join(VIDEO, '*.ts')))
    for i in range(len(segs) - 1, -1, -1):
        start = datetime.datetime.strptime(os.path.basename(segs[i])[:15], '%Y%m%d_%H%M%S').timestamp()
        if start <= wall:
            return 'concat:' + '|'.join(segs[i:i + 2]), wall - start
    return None


def strip(wall, out):
    """Full-resolution frames of the Hive from just before the release to its landing: a 6x3 sheet at 15 fps and
    a 4x slow clip next to it."""
    seg = segment_for(wall - 0.2)
    if not seg:
        return None
    f, off = seg
    tmp = out + '.d'
    os.makedirs(tmp, exist_ok=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', '%.2f' % off, '-i', f, '-t', '1.2', '-vf',
                    'fps=15,' + CROP, os.path.join(tmp, 'f_%02d.png')], check=False)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', os.path.join(tmp, 'f_%02d.png'), '-frames:v', '1', '-vf',
                    "drawtext=text='%{frame_num}':start_number=1:x=8:y=8:fontsize=32:fontcolor=yellow,tile=6x3,"
                    "scale=iw*0.55:-1", '-q:v', '3', out], check=False)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', '%.2f' % off, '-i', f, '-t', '1.5', '-vf',
                    CROP + ',setpts=4*PTS', '-an', '-c:v', 'libx264', '-crf', '20', out[:-4] + '.mp4'], check=False)
    for x in glob.glob(os.path.join(tmp, '*')):
        os.remove(x)
    os.rmdir(tmp)
    return out if os.path.exists(out) else None


def main():
    path = sys.argv[1]
    name = os.path.basename(path)[:-5]
    offset = float(open(path + '.clock').read())
    d, _ = parse(path)

    def s(k):
        return d.get(R + k, [])

    def at(k, t):
        v = [b for a, b in s(k) if a <= t]
        return v[-1] if v else None

    grid = at('FieldTest/OnlyGrid', 1e12)
    alliance = at('FieldTest/Alliance', 1e12)
    dips = [t for t, v in s('Feeder/BallLeft') if v]
    feeds, prev = [], None
    for t, v in s('Feeder/State'):
        if v == 'FEED' and prev != 'FEED' and at('FieldTest/Phase', t) in ('PRE_AIM', 'AFTER'):
            feeds.append(t)
        prev = v
    # One ball per feed: the first dip after it and before the next feed
    balls = []
    for i, f in enumerate(feeds):
        nxt = feeds[i + 1] if i + 1 < len(feeds) else 1e12
        dip = next((t for t in dips if f <= t < min(nxt, f + 0.8)), None)
        balls.append((f, dip if dip is not None else f + NO_DIP_DELAY_S, dip is not None))
    balls = balls[:int(sys.argv[2]) if len(sys.argv) > 2 else 3]
    os.makedirs(OUT, exist_ok=True)
    done = set()
    if os.path.exists(CSV):
        done = {(r['log'], r['ball']) for r in csv.DictReader(open(CSV))}
    new = not os.path.exists(CSV)
    w = csv.DictWriter(open(CSV, 'a', newline=''), FIELDS)
    if new:
        w.writeheader()
    bv = [b for a, b in s('Robot/BatteryVolts') if b == b]
    print('%s  %s grid %s  battery %.1f-%.1f V  finish: %s' % (name, alliance, grid, min(bv), max(bv),
                                                               at('FieldTest/FinishReason', 1e12)))
    for i, (f, t, dipped) in enumerate(balls):
        pose = at('Odometry/Pose', t)
        cx, cy = at('SOTM/CellX', t), at('SOTM/CellY', t)
        off = math.degrees(math.atan2(abs(pose[1] - cy), abs(pose[0] - cx)))
        row = dict(log=name, ball=str(i + 1), alliance=alliance, grid=grid, x='%.2f' % pose[0], y='%.2f' % pose[1],
                   lip_m='%.3f' % at('SOTM/DistanceToLipM', t), off_deg='%.0f' % off,
                   hood='%.1f' % at('Hood/AngleDeg', t), hood_tgt='%.1f' % at('Hood/TargetAngleDeg', t),
                   rpm='%.0f' % at('Shooter/Rpm', f + 0.15), rpm_tgt='%.0f' % at('Shooter/TargetRpm', t),
                   head_err_deg='%+.1f' % wrap_deg(at('Drive/HeadingRad', t) - at('SOTM/LockHeadingRad', t)),
                   delay_s='%.2f' % (t - f), dip='1' if dipped else '0',
                   wall=datetime.datetime.fromtimestamp(t + offset).strftime('%H:%M:%S.%f')[:-4], result='')
        img = strip(t + offset, os.path.join(OUT, '%s_ball%d.jpg' % (name, i + 1)))
        print('ball %d %s lip %s m off %s deg hood %s/%s rpm %s/%s heading %s delay %s%s  %s' % (
            i + 1, row['wall'], row['lip_m'], row['off_deg'], row['hood'], row['hood_tgt'], row['rpm'],
            row['rpm_tgt'], row['head_err_deg'], row['delay_s'], '' if dipped else ' (no dip)', img or 'no video'))
        if (name, str(i + 1)) not in done:
            w.writerow(row)


if __name__ == '__main__':
    main()
