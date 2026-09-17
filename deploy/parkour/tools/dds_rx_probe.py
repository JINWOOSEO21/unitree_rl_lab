#!/usr/bin/env python3
"""Receive-only DDS probe for the Go2 migration (plan stages C-1..C-3, D-2, D-4).

Creates subscribers only: no publisher, no LowCmd, no sport client, nothing is sent to the robot beyond
DDS discovery. Reports per-topic rate and inter-arrival gaps, LowState tick continuity, and a wall +
monotonic anchor so logs from hosts with unsynchronised clocks (the Jetson reads 1970) can be aligned on
the shared LowState tick instead of on wall time.

Usage: python tools/dds_rx_probe.py --interface eth0 --duration 30 [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time

TOPICS = {
    # topic: (module, type name, expected Hz or None)
    'rt/lowstate': ('unitree_sdk2py.idl.unitree_go.msg.dds_', 'LowState_', 500.0),
    'rt/utlidar/cloud': ('unitree_sdk2py.idl.sensor_msgs.msg.dds_', 'PointCloud2_', None),
    'rt/utlidar/robot_odom': ('unitree_sdk2py.idl.nav_msgs.msg.dds_', 'Odometry_', None),
    'rt/parkour/scandots': ('unitree_sdk2py.idl.unitree_go.msg.dds_', 'HeightMap_', 10.0),
    'rt/parkour/gyro_bias': ('unitree_sdk2py.idl.std_msgs.msg.dds_', 'String_', None),
}


def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values) - 1, int(len(sorted_values) * q))]


def summarize(name, stamps, ticks, duration):
    out = {'topic': name, 'count': len(stamps)}
    if not stamps:
        out['status'] = 'NOT RECEIVED'
        return out
    gaps = sorted((b - a) / 1e6 for a, b in zip(stamps, stamps[1:]))
    out['rate_hz'] = round(len(stamps) / duration, 2)
    out['first_after_s'] = round(stamps[0] / 1e9, 3)
    if gaps:
        out['gap_ms'] = {'p50': round(percentile(gaps, .5), 2), 'p95': round(percentile(gaps, .95), 2),
                         'p99': round(percentile(gaps, .99), 2), 'max': round(gaps[-1], 2)}
    if ticks:
        steps = [b - a for a, b in zip(ticks, ticks[1:])]
        out['tick'] = {'first': ticks[0], 'last': ticks[-1],
                       'backwards': sum(s < 0 for s in steps), 'repeated': sum(s == 0 for s in steps),
                       'max_step': max(steps) if steps else 0}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--interface', required=True, help='NIC on the robot network, e.g. eth0 / enp42s0 / wlo1')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--topics', nargs='*', default=list(TOPICS))
    parser.add_argument('--json', help='write the full summary here')
    args = parser.parse_args()

    import importlib
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

    anchor = {'host': socket.gethostname(), 'interface': args.interface, 'domain': args.domain,
              'wall_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
              'wall_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    ChannelFactoryInitialize(args.domain, args.interface)
    t0 = time.monotonic_ns()
    lock = threading.Lock()
    stamps = {t: [] for t in args.topics}
    ticks = []
    subs = []

    def make_callback(topic):
        store = stamps[topic]
        if topic == 'rt/lowstate':
            def callback(msg):
                now = time.monotonic_ns() - t0
                with lock:
                    store.append(now)
                    ticks.append(int(msg.tick))
        else:
            def callback(_msg):
                now = time.monotonic_ns() - t0
                with lock:
                    store.append(now)
        return callback

    for topic in args.topics:
        module, type_name, _ = TOPICS[topic]
        sub = ChannelSubscriber(topic, getattr(importlib.import_module(module), type_name))
        sub.Init(make_callback(topic), 0)  # same immediate-callback mode as go2_sensor_bridge.py
        subs.append(sub)

    print(f"receive-only probe on {args.interface} domain {args.domain} for {args.duration:.0f}s "
          f"(no publishers created)", flush=True)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        time.sleep(min(5.0, max(0.0, end - time.monotonic())))
        with lock:
            print('  ' + '  '.join(f"{t.split('/', 1)[1]}={len(v)}" for t, v in stamps.items()), flush=True)

    elapsed = (time.monotonic_ns() - t0) / 1e9
    with lock:
        results = [summarize(t, list(stamps[t]), list(ticks) if t == 'rt/lowstate' else None, elapsed)
                   for t in args.topics]
    for sub in subs:
        sub.Close()
    report = {'anchor': anchor, 'elapsed_s': round(elapsed, 3), 'topics': results}
    for r in results:
        gap = r.get('gap_ms', {})
        print(f"{r['topic']:26s} n={r['count']:<7d} "
              + (r.get('status') or f"{r['rate_hz']:8.2f} Hz  gap ms p50 {gap.get('p50')} p95 {gap.get('p95')} "
                                    f"max {gap.get('max')}  first {r['first_after_s']}s")
              + (f"  tick {r['tick']}" if 'tick' in r else ''))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(report, f, indent=2)
        print('saved', args.json)


if __name__ == '__main__':
    main()
