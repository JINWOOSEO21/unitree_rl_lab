"""Why the bridge stops consuming rt/utlidar/cloud after ~22 s. Subscriber only.

The bridge drops a cloud silently when its header stamp does not advance, and rejects it
as cloud_too_old when the tick loop reaches it more than 200 ms after arrival. Both are
invisible in the bridge's own log, so measure them here: stamp continuity, arrival gaps,
point count, and -- with --map -- what a real cloud actually costs on this GPU, which is
the number plan C-5 asks for before anyone touches a timeout.

Usage: python3 cloud_probe.py --parkour <dir> [--emcupy <dir> --map] [--duration 60]
"""
from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--parkour', required=True)
    ap.add_argument('--emcupy')
    ap.add_argument('--interface', default='eth0')
    ap.add_argument('--domain', type=int, default=0)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--map', action='store_true', help='also time a real map update per cloud')
    args = ap.parse_args()

    sys.path.insert(0, args.parkour)
    sys.path.insert(0, args.parkour + '/tools')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    lock = threading.Lock()
    state = {'n': 0, 'same_stamp': 0, 'back_stamp': 0, 'last_stamp': None,
             'last_at': None, 'low': 0}
    arrivals: list[float] = []   # inter-arrival ms
    stamp_steps: list[float] = []  # header stamp advance ms
    latest = [None]

    def on_cloud(msg):
        now = time.monotonic()
        stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        with lock:
            state['n'] += 1
            previous, previous_at = state['last_stamp'], state['last_at']
            if previous is not None:
                if stamp == previous:
                    state['same_stamp'] += 1   # the bridge drops these without a word
                    return
                if stamp < previous:
                    state['back_stamp'] += 1
                    return
                stamp_steps.append((stamp-previous)/1e6)
                arrivals.append((now-previous_at)*1e3)
            state['last_stamp'], state['last_at'] = stamp, now
            latest[0] = (now, msg)

    def on_low(msg):
        with lock:
            state['low'] += 1

    ChannelFactoryInitialize(args.domain, args.interface)
    cloud_sub = ChannelSubscriber('rt/utlidar/cloud', PointCloud2_)
    cloud_sub.Init(on_cloud, 0)
    low_sub = ChannelSubscriber('rt/lowstate', LowState_)
    low_sub.Init(on_low, 0)  # reproduce the bridge's 500 Hz GIL load

    mapper = None
    map_lat: list[float] = []
    points: list[int] = []
    age_at_tick: list[float] = []
    if args.map:
        import torch
        from go2_sensor_bridge import BaseMapper
        from em_sidecar.go2_cloud import raw_cloud_to_base
        mapper = BaseMapper(args.emcupy)
        mapper.warmup()
        row = {'frame_id': 'odom', 'child_frame_id': 'base_link',
               'position': {'x': 0.0, 'y': 0.0, 'z': 0.32},
               'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}}

    print(f'cloud probe: {args.duration:.0f}s map={"on" if args.map else "off"} '
          f'(subscriber only, nothing sent)', flush=True)

    start = time.monotonic()
    next_tick = start
    next_report = start + 10.0
    last_seen = None
    while time.monotonic()-start < args.duration:
        time.sleep(max(0.0, next_tick-time.monotonic()))
        next_tick = max(next_tick+0.1, time.monotonic())
        with lock:
            item = latest[0]
        if item is not None and item is not last_seen:
            last_seen = item
            receipt, msg = item
            # Exactly the bridge's check: how stale is the cloud when the tick reaches it?
            age_at_tick.append((time.monotonic()-receipt)*1e3)
            if mapper is not None:
                base = raw_cloud_to_base(msg)
                points.append(len(base))
                t0 = time.perf_counter()
                try:
                    mapper.update(None, row, base_points=base)
                except Exception as exc:
                    print(f'  map update rejected: {type(exc).__name__}: {exc}', flush=True)
                torch.cuda.synchronize()
                map_lat.append((time.perf_counter()-t0)*1e3)
        if time.monotonic() >= next_report:
            with lock:
                n, same, back = state['n'], state['same_stamp'], state['back_stamp']
            print(f'  t={time.monotonic()-start:5.0f}s clouds={n} same_stamp={same} '
                  f'back_stamp={back} ticked={len(age_at_tick)} '
                  f'map_n={len(map_lat)} low={state["low"]}', flush=True)
            next_report += 10.0

    cloud_sub.Close()
    low_sub.Close()

    def show(name, values, unit='ms'):
        if not values:
            print(f'  {name}: none')
            return
        s = sorted(values)
        print(f'  {name}: n={len(s)} p50={s[len(s)//2]:.1f} p90={s[int(len(s)*.9)]:.1f} '
              f'max={s[-1]:.1f} mean={statistics.fmean(s):.1f} {unit}')

    print('--- summary ---')
    print(f'  clouds={state["n"]} same_stamp={state["same_stamp"]} '
          f'back_stamp={state["back_stamp"]} lowstate={state["low"]}')
    show('cloud inter-arrival', arrivals)
    show('header stamp step', stamp_steps)
    show('age when the tick reaches it (>200 = cloud_too_old)', age_at_tick)
    if mapper is not None:
        show('real map update', map_lat)
        show('points per cloud', points, 'pts')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
