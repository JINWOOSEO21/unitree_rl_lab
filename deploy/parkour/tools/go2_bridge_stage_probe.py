"""Which stage of the bridge stops? Run the real bridge and count every stage it passes.

The bridge goes silent after ~22 s: no scan, no fault, while lowstate keeps flowing. Its own
log cannot say why, because the stages that could be stalling do not emit anything. So wrap
the module-level functions each stage must call -- they are only reachable from one place
each -- and report the counters every second:

  low_row        entered the 500 Hz LowState callback
  stamp_id       entered the cloud/odom callback          <- stops if the reader thread dies
  sleep          the 10 Hz tick loop went around          <- stops if the loop is stuck
  preceding_pose the loop accepted a new cloud and is pairing it with a pose
  map_update     the loop reached the GPU

time.sleep is the loop's heartbeat because the loop calls it exactly once per iteration and
nothing else in the bridge calls it at all. (time.monotonic would not do: emit() calls it
once per LowState event, so 500 Hz of noise would bury the loop's 10 Hz.) Patching it does
reach any other library that sleeps, so cross-check a suspicious count against --stacks.

Nothing here changes the bridge's behaviour; every wrapper calls straight through. With
--stacks it also dumps all thread stacks periodically, which names the exact line a stalled
thread is sitting on.

Usage: python3 bridge_stages.py --parkour <dir> -- <go2_sensor_bridge.py args...>
"""
from __future__ import annotations

import argparse
import faulthandler
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--parkour', required=True)
    ap.add_argument('--stacks', type=float, default=0.0,
                    help='dump every thread stack this often (seconds); 0 disables')
    ap.add_argument('rest', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    bridge_argv = args.rest[1:] if args.rest and args.rest[0] == '--' else args.rest

    sys.path.insert(0, args.parkour)
    sys.path.insert(0, args.parkour + '/tools')
    import go2_sensor_bridge as bridge

    counts = {'low_row': 0, 'stamp_id': 0, 'sleep': 0,
              'preceding_pose': 0, 'map_update': 0}
    slow = {'map_update': 0.0, 'preceding_pose': 0.0}
    lock = threading.Lock()

    def counted(name, function):
        def wrapper(*a, **kw):
            with lock:
                counts[name] += 1
            return function(*a, **kw)
        return wrapper

    def timed(name, function):
        def wrapper(*a, **kw):
            started = time.perf_counter()
            try:
                return function(*a, **kw)
            finally:
                elapsed = (time.perf_counter()-started)*1e3
                with lock:
                    counts[name] += 1
                    slow[name] = max(slow[name], elapsed)
        return wrapper

    bridge.low_row = counted('low_row', bridge.low_row)
    bridge.stamp_id = counted('stamp_id', bridge.stamp_id)
    bridge.preceding_pose = timed('preceding_pose', bridge.preceding_pose)
    bridge.BaseMapper.update = timed('map_update', bridge.BaseMapper.update)
    # The tick loop calls sleep exactly once per iteration: its own heartbeat.
    bridge.time.sleep = counted('sleep', bridge.time.sleep)

    stop = threading.Event()

    def report():
        start = time.time()
        previous = dict(counts)
        while not stop.wait(1.0):
            with lock:
                now = dict(counts)
                peaks = dict(slow)
            delta = {k: now[k]-previous[k] for k in now}
            previous = now
            print(f'[stage] t={time.time()-start:5.1f}s '
                  f'low={now["low_row"]:6d}(+{delta["low_row"]:4d}) '
                  f'cb={now["stamp_id"]:5d}(+{delta["stamp_id"]:3d}) '
                  f'tick={now["sleep"]:6d}(+{delta["sleep"]:4d}) '
                  f'pair={now["preceding_pose"]:4d}(+{delta["preceding_pose"]:2d}) '
                  f'map={now["map_update"]:4d}(+{delta["map_update"]:2d}) '
                  f'| map_max={peaks["map_update"]:.0f}ms pair_max={peaks["preceding_pose"]:.0f}ms',
                  file=sys.stderr, flush=True)

    threading.Thread(target=report, daemon=True).start()
    if args.stacks > 0:
        faulthandler.dump_traceback_later(args.stacks, repeat=True, exit=False,
                                          file=sys.stderr)

    sys.argv = ['go2_sensor_bridge.py'] + bridge_argv
    try:
        bridge.main()
    except SystemExit as exit_code:
        return int(exit_code.code or 0)
    finally:
        stop.set()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
