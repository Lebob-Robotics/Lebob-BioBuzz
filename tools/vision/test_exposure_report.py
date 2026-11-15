"""Checks exposure_report on a synthetic log carrying the brightness and exposure keys the robot now writes."""
import io
import os
import struct
import sys
import tempfile
import contextlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import exposure_report as er


class Writer:
    def __init__(self):
        self.buf = bytearray(b'\x01')
        self.ids = {}

    def time(self, t):
        self.buf += b'\x00' + struct.pack('>d', t)

    def put(self, key, typ, payload):
        if key not in self.ids:
            kid = len(self.ids)
            self.ids[key] = kid
            n, ty = key.encode(), typ.encode()
            self.buf += b'\x01' + struct.pack('>hh', kid, len(n)) + n + struct.pack('>h', len(ty)) + ty
        self.buf += b'\x02' + struct.pack('>hH', self.ids[key], len(payload)) + payload


def d(v): return struct.pack('>d', v)
def q(v): return struct.pack('>q', v)
def da(vs): return struct.pack('>%dd' % len(vs), *vs)
def qa(vs): return struct.pack('>%dq' % len(vs), *vs)


def build(mean, clip):
    w = Writer()
    w.put('RealMetadata/OpMode Name', 'string', b'Lebot Teleop')
    for k in range(200):  # 20 s at 10 Hz, robot still
        t = k * 0.1
        w.time(t)
        w.put('RealOutputs/Odometry/Pose', 'struct:Pose2d', struct.pack('<3d', 0, 0, 0))
        for side in ('Left', 'Right'):
            p = f'Vision/{side}/'
            w.put('RealOutputs/' + p + 'Lighting', 'string', b'COMP')
            w.put('RealOutputs/' + p + 'ExposureMs', 'int64', q(3))
            w.put('RealOutputs/' + p + 'Gain', 'int64', q(30))
            w.put('RealOutputs/' + p + 'FramesTotal', 'int64', q(k * 2))
            w.put('RealOutputs/' + p + 'Brightness/Mean', 'double', d(mean))
            w.put('RealOutputs/' + p + 'Brightness/ClippedPct', 'double', d(clip))
            w.put('RealOutputs/' + p + 'Brightness/DarkPct', 'double', d(2.0))
            w.put(p + 'Fps', 'double', d(20.0))
            w.put(p + 'TagIds', 'int64[]', qa([30, 31]))
            w.put(p + 'XIn', 'double[]', da([0, 5]))
            w.put(p + 'YIn', 'double[]', da([40, 40]))  # About 1 m
            w.put(p + 'ZIn', 'double[]', da([0, 0]))
            w.put(p + 'DecisionMargin', 'double[]', da([90 + k % 5, 95]))
            w.put(p + 'FrameNanos', 'int64[]', qa([k * 100_000_000] * 2))
    f = tempfile.NamedTemporaryFile(suffix='.rlog', delete=False)
    f.write(w.buf)
    f.close()
    return f.name


def run(path):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        er.main([path])
    return out.getvalue()


def test_bright_log_says_lower_gain():
    out = run(build(mean=185, clip=22))
    assert 'field lighting preset COMP' in out, out
    assert 'mean grey 185/255' in out and 'clipped white 22.0%' in out, out
    assert 'too bright: lower gain to 20' in out, out
    assert '1 of frames' not in out and '50% of frames' in out, out  # 200 tagged frames of ~398


def test_good_log_says_keep():
    out = run(build(mean=110, clip=1))
    assert 'brightness fine: keep it' in out, out


if __name__ == '__main__':
    test_bright_log_says_lower_gain()
    test_good_log_says_keep()
    print('ok')
