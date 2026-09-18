"""LowState gap spike probe: does the bridge workload recover after a spike?

Subscriber only. No publisher, no LowCmd, no service client. Nothing is sent to the robot.

Written when go2_leg_pose.update() raised on a tick gap over max_dt_s without ever advancing
last_tick, so the bridge died permanently on the first spike and the tail past it had never
been observed. That wedge is gone (the estimator now recovers through resume_after_gap), but
the measurement still answers the standing question: under the real load -- rt/lowstate at
500 Hz plus a 10 Hz elevation-map update holding the GIL -- how large do the gaps get, and
does the stream return to 2 ms immediately afterwards? This probe only records, never wedges.

Usage:
  python go2_lowstate_gap_probe.py --parkour <dir> --emcupy <dir> --interface eth0 \
      --duration 120 [--no-map]
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--parkour', required=True)
    ap.add_argument('--emcupy', required=True)
    ap.add_argument('--interface', default='eth0')
    ap.add_argument('--domain', type=int, default=0)
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--no-map', action='store_true', help='measure without the map load')
    ap.add_argument('--spike-ms', type=float, default=20.0)
    ap.add_argument('--json', help='write the summary here')
    args = ap.parse_args()

    sys.path.insert(0, args.parkour)
    sys.path.insert(0, args.parkour + '/tools')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    lock = threading.Lock()
    gaps: list[tuple[float, int, float]] = []  # (monotonic_s, tick, gap_ms)
    state = {'last_tick': None, 'n': 0, 'dup': 0, 'regress': 0}

    def on_low(msg):
        now = time.monotonic()
        tick = int(msg.tick)
        with lock:
            previous = state['last_tick']
            state['last_tick'] = tick  # always advance: never wedge like go2_leg_pose does
            state['n'] += 1
            if previous is None:
                return
            delta = tick - previous
            if delta == 0:
                state['dup'] += 1
                return
            if delta < 0:
                state['regress'] += 1
                return
            gaps.append((now, tick, float(delta)))

    ChannelFactoryInitialize(args.domain, args.interface)
    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(on_low, 0)  # same as the bridge: callback-time sampling, no queued delivery

    mapper = None
    map_lat: list[float] = []
    if not args.no_map:
        import numpy as np
        import torch
        from go2_sensor_bridge import BaseMapper

        mapper = BaseMapper(args.emcupy)
        row = {'frame_id': 'odom', 'child_frame_id': 'base_link',
               'position': {'x': 0.0, 'y': 0.0, 'z': 0.3},
               'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}}
        rng = np.random.default_rng(0)
        xy = rng.uniform(-1.5, 1.5, size=(6000, 2))
        z = np.where((xy[:, 0] > 0.6) & (xy[:, 0] < 1.0), -0.15, -0.30)
        pts = np.column_stack([xy, z]).astype(np.float32)

    print(f'gap probe: interface={args.interface} domain={args.domain} '
          f'duration={args.duration:.0f}s map={"off" if args.no_map else "on"} '
          f'(subscriber only, nothing sent)', flush=True)

    start = time.monotonic()
    next_map = start
    next_report = start + 10.0
    while time.monotonic() - start < args.duration:
        if mapper is not None and time.monotonic() >= next_map:
            row['position']['x'] += 0.002
            t0 = time.perf_counter()
            mapper.update(None, row, base_points=pts)
            torch.cuda.synchronize()
            map_lat.append((time.perf_counter() - t0) * 1e3)
            next_map = max(next_map + 0.1, time.monotonic())
        else:
            time.sleep(0.002)
        if time.monotonic() >= next_report:
            with lock:
                spikes = [g for g in gaps if g[2] > args.spike_ms]
                worst = max((g[2] for g in gaps), default=0.0)
            print(f'  t={time.monotonic()-start:5.0f}s samples={state["n"]} '
                  f'spikes>{args.spike_ms:.0f}ms={len(spikes)} worst={worst:.0f}ms '
                  f'map_n={len(map_lat)}', flush=True)
            next_report += 10.0

    sub.Close()
    with lock:
        values = [g[2] for g in gaps]
        spikes = [g for g in gaps if g[2] > args.spike_ms]

    if not values:
        print('NO DATA: no lowstate received.', file=sys.stderr)
        return 2
    values_sorted = sorted(values)

    def pct(p: float) -> float:
        return values_sorted[min(len(values_sorted) - 1, int(len(values_sorted) * p))]

    # The question this probe exists to answer: after each spike, does the stream return to
    # normal immediately? Report the gaps that follow every spike.
    recovery = []
    index = {id(g): i for i, g in enumerate(gaps)}
    for g in spikes:
        i = index[id(g)]
        following = [round(x[2], 1) for x in gaps[i + 1:i + 11]]
        recovery.append({'at_s': round(g[0] - start, 2), 'tick': g[1],
                         'gap_ms': g[2], 'next_10_gaps_ms': following})

    over_100 = [g for g in values if g > 100.0]
    summary = {
        'samples': state['n'], 'intervals': len(values), 'duplicates': state['dup'],
        'regressions': state['regress'], 'map_updates': len(map_lat),
        'gap_ms': {'p50': pct(.50), 'p90': pct(.90), 'p99': pct(.99), 'max': values_sorted[-1],
                   'mean': round(statistics.fmean(values), 3)},
        'spikes_over_%.0fms' % args.spike_ms: len(spikes),
        'gaps_over_100ms': len(over_100),
        'map_ms': ({'p50': round(sorted(map_lat)[len(map_lat) // 2], 1),
                    'max': round(max(map_lat), 1)} if map_lat else None),
        'recovery_after_spikes': recovery[:40],
    }
    print(json.dumps(summary, indent=2), flush=True)
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(summary, f, indent=2)
        print('saved', args.json, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
