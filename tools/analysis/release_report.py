#!/usr/bin/env python3
"""How long a ball takes from the feed starting to leaving the shooter, from robot logs.

Shoot on the move aims for where the robot will be when the ball leaves: AimConstants.FEED_DELAY_S ahead. This
measures that delay on real balls: from Feeder/State turning FEED to the first Shooter/Rpm sample of the dip the
ball makes going through the wheels (the wheels' reading drops as the ball touches them).

    python3 tools/analysis/release_report.py logs/Main_log_2026*.rlog logs/VolleyTuning_log_*.rlog

Prints the delay's spread over all balls, and split into the first ball of a press and the ones after.
FEED_DELAY_S is the median plus the age of the pose the aim acts on (about one loop, ~50 ms): 0.25 + 0.05 measured
on 15-19 Nov logs, which is the 0.30 in AimConstants. p10-p90 is the spread readyToFire's FEED_DELAY_MAX_S covers.
"""
import bisect
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rlogparse import parse  # noqa: E402

BASE_MIN_RPM = 2000   # Feeds with the wheels slower than this are not shots
DIP_RPM = 150         # A ball's dip is at least this deep
DIP_START_RPM = 40    # The dip starts where the reading first falls this far
MAX_DELAY_S = 1.2
FIRST_GAP_S = 1.5     # A feed this long after the last starts a new press


def releases(path):
    d, _ = parse(path)
    out = lambda k: d.get('RealOutputs/' + k, [])
    state, rpm = out('Feeder/State'), out('Shooter/Rpm')
    if not rpm:
        return []
    times = [t for t, _ in rpm]
    feeds = [t for i, (t, v) in enumerate(state) if v == 'FEED' and (i == 0 or state[i - 1][1] != 'FEED')]
    result, last = [], -1e9
    for t0 in feeds:
        i0 = bisect.bisect_left(times, t0)
        before = [v for t, v in rpm[max(0, i0 - 8):i0] if t > t0 - 0.25]
        if len(before) < 2 or statistics.median(before) < BASE_MIN_RPM:
            continue
        base = statistics.median(before)
        j = next((k for k in range(i0, len(rpm)) if rpm[k][0] - t0 > MAX_DELAY_S or rpm[k][1] < base - DIP_RPM),
                 None)
        if j is None or rpm[j][0] - t0 > MAX_DELAY_S:
            continue
        k = j
        while k > i0 and rpm[k - 1][1] < base - DIP_START_RPM:
            k -= 1
        result.append((rpm[k][0] - t0, t0 - last > FIRST_GAP_S))
        last = t0
    return result


def summary(name, delays):
    if len(delays) < 3:
        print(f'{name}: {len(delays)} balls, too few')
        return
    delays = sorted(delays)
    pct = lambda p: delays[min(len(delays) - 1, int(p * len(delays)))]
    print(f'{name}: {len(delays)} balls  median {statistics.median(delays):.3f} s  '
          f'p10 {pct(0.1):.3f}  p90 {pct(0.9):.3f}')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    every = []
    for path in sys.argv[1:]:
        try:
            every += releases(path)
        except Exception as e:  # A cut-off log
            print(f'skipped {path}: {e}')
    summary('all balls', [d for d, _ in every])
    summary('first of a press', [d for d, first in every if first])
    summary('later balls', [d for d, first in every if not first])
    print('Each is rounded up to the next loop (~50 ms). FEED_DELAY_S = median + pose age (~0.05 s).')


if __name__ == '__main__':
    main()
