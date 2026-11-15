#!/usr/bin/env python3
"""autodual.py

Watches the hub and makes a side-by-side video (dual.py) of every OpMode run as it finishes: tuning, calibration,
tests, TeleOp. For each new PsiKit log it waits until the log stops growing, pulls it into logs/ with LOG.clock
(PC clock minus hub uptime, exact because the log is from the running boot), pulls the robot camera frames of that
run into robotcam/, renders /var/mnt/timelapse/dual/<wall time>_<OpMode>.mp4, and for SotmBallTest/SotmFieldTest
also the ball report. Logs already on the hub when it starts are left alone (their boot, and so their clock, is
unknown). Robot camera frames are only pulled while no OpMode is running, so the Wi-Fi stays free for driving.

Runs until killed; output goes to autodual.log next to it. Rendering runs one video at a time in the background
while it keeps watching."""
import os, sys, time, json, subprocess, threading, queue, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
HUB = '192.168.43.1:5555'
LOGS = os.path.join(HERE, 'logs')
CAM = os.path.join(HERE, 'robotcam')
BUSY = '/tmp/hub_busy'
PY = os.path.join(HERE, '.venv', 'bin', 'python')
STATE = os.path.join(HERE, 'autodual_state.json')


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def adb(*args, timeout=30):
    r = subprocess.run(['adb', '-s', HUB] + list(args), capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError((r.stderr or r.stdout).strip()[:200])
    return r.stdout


def hub_list(folder, pattern='*'):
    """{name: (size, mtime)}, the hub's own clock and its boot id (first 8 characters), in one shell call."""
    out = adb('shell', 'cat /proc/sys/kernel/random/boot_id; date +%%s; stat -c "%%s %%Y %%n" %s/%s 2>/dev/null'
              % (folder, pattern))
    lines = out.strip().splitlines()
    boot = lines[0].strip()[:8]
    now = int(lines[1])
    files = {}
    for l in lines[2:]:
        parts = l.split(' ', 2)
        if len(parts) == 3 and parts[0].isdigit():
            files[os.path.basename(parts[2])] = (int(parts[0]), int(parts[1]))
    return now, files, boot


def local_name(name, boot):
    """The hub names logs by its clock, which restarts at 1970 each boot, so names repeat across boots."""
    return name[:-len('.rlog')] + '_b' + boot + '.rlog'


def clock_offset():
    a = time.time()
    up = float(adb('shell', 'cat /proc/uptime').split()[0])
    b = time.time()
    return (a + b) / 2 - up


def log_span(path):
    sys.path.insert(0, HERE)
    from rlogparse import parse
    d, _ = parse(path)
    ts = [t for k, v in d.items() if k.startswith('RealOutputs/') and v for t in (v[0][0], v[-1][0])]
    return (min(ts), max(ts)) if ts else (None, None)


def notify(msg):
    if shutil.which('notify-send'):
        subprocess.run(['notify-send', '-a', 'BioBuzz', 'Run video', msg], capture_output=True)


def render_worker(jobs):
    while True:
        path = jobs.get()
        name = os.path.basename(path)
        try:
            r = subprocess.run([PY, os.path.join(HERE, 'dual.py'), path], capture_output=True, text=True)
            out = (r.stdout.strip().splitlines() or ['?'])[-1]
            say('video', name, '->', out if r.returncode == 0 else 'FAILED ' + r.stderr[-300:])
            if r.returncode == 0:
                notify(out)
            if name.startswith(('SotmBallTest', 'SotmFieldTest')):
                r = subprocess.run([PY, os.path.join(HERE, 'ballreport.py'), path], capture_output=True, text=True)
                say('ball report', name, 'ok' if r.returncode == 0 else 'failed: ' + r.stderr[-200:])
        except Exception as e:
            say('render failed', name, e)


def main():
    os.makedirs(LOGS, exist_ok=True)
    os.makedirs(CAM, exist_ok=True)
    state = json.load(open(STATE)) if os.path.exists(STATE) else {}
    jobs = queue.Queue()
    threading.Thread(target=render_worker, args=(jobs,), daemon=True).start()
    known = None          # Logs present when watching began, or done
    sizes = {}            # name@boot -> (size, polls unchanged)
    pending_cams = []     # (log path, start, end) waiting for an idle hub
    online = None
    while True:
        # balltest.sh marks a run in progress: listing and pulling during one stalled the robot loop 0.5-0.7 s
        # (6 Nov, 081827)
        if os.path.exists(BUSY) and time.time() - os.path.getmtime(BUSY) < 600:
            time.sleep(2)
            continue
        try:
            now, files, boot = hub_list('/sdcard/FIRST/PsiKit', '*.rlog')
            if online is not True:
                say('hub online')
                online = True
        except Exception as e:
            if online is not False:
                say('hub offline (%s)' % e)
                online = False
            time.sleep(5)
            continue
        if known is None:
            known = {n + '@' + boot for n in files} | set(state.get('done', []))
            say('watching; %d logs already on the hub left alone' % len(files))
        growing = False
        for name, (size, mtime) in sorted(files.items(), key=lambda kv: kv[1][1]):
            key = name + '@' + boot
            if key in known:
                continue
            prev = sizes.get(key, (-1, 0))
            sizes[key] = (size, prev[1] + 1 if size == prev[0] else 0)
            if sizes[key][1] < 3 or now - mtime < 5:
                growing = True
                continue
            local = os.path.join(LOGS, local_name(name, boot))
            try:
                if not (os.path.exists(local) and os.path.getsize(local) == size):
                    adb('pull', '/sdcard/FIRST/PsiKit/' + name, local, timeout=120)
                if not os.path.exists(local + '.clock'):
                    open(local + '.clock', 'w').write('%.3f\n' % clock_offset())
                a, b = log_span(local)
            except Exception as e:
                say('pull failed', name, e)
                continue
            known.add(key)
            state.setdefault('done', []).append(key)
            json.dump(state, open(STATE, 'w'))
            if a is None or b - a < 3:
                say('skipped', name, '(under 3 s)')
                continue
            say('new run', name, '%.0f s' % (b - a))
            pending_cams.append((local, a, b))
        if pending_cams and not growing:
            try:
                names = adb('shell', 'ls /sdcard/FIRST/robotcam 2>/dev/null').split()
            except Exception:
                names = []
            for local, a, b in pending_cams:
                for dname in names:
                    try:
                        start = int(dname.split('_')[0])
                    except ValueError:
                        continue
                    if a - 60 <= start <= b:
                        try:
                            adb('pull', '/sdcard/FIRST/robotcam/' + dname, CAM + '/', timeout=600)
                        except Exception as e:
                            say('robot camera pull failed', dname, e)
                jobs.put(local)
            pending_cams = []
        time.sleep(3)


if __name__ == '__main__':
    main()
