"""Does running the leg estimator below the LowState rate change what it estimates?

Subscriber only: records rt/lowstate, then replays the one recording through LegPose at
several rates offline. Nothing is sent to the robot and no map or GPU is involved.

Why the question is not obvious. Two things in LegOdomCfg are tied to the sample interval:

  contact_settle_s = 0.02  a foot must hold contact this long before it is trusted, so the
                           interval has to resolve 20 ms -- 100 Hz gives two samples, 50 Hz one.
  force_lp_tau_s   = 0.006 the contact-force low pass. Its gain is min(1, dt/tau), so it is
                           active at 500 Hz (0.33) and fully disabled at any rate at or below
                           167 Hz. The offline gate (em_sidecar/tests/test_leg_odometry.py)
                           drives the estimator at 50 Hz, so the rate this deployment wants
                           puts that filter in the same state the gate already validates.

What is left to check is the real robot: decimation must not alias the foot-force chatter
into lost contacts, and must not add drift while standing. That is what this measures.

Usage: python3 go2_leg_rate_compare.py --parkour <dir> [--interface eth0] [--duration 30]
"""
from __future__ import annotations

import argparse
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--parkour', required=True)
    ap.add_argument('--interface', default='eth0')
    ap.add_argument('--domain', type=int, default=0)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--rates', default='0,200,100,50',
                    help='estimator rates to compare; 0 means every sample')
    args = ap.parse_args()

    sys.path.insert(0, args.parkour)
    sys.path.insert(0, args.parkour + '/tools')
    import numpy as np
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from go2_leg_pose import LegPose
    from go2_sensor_bridge import low_row

    lock = threading.Lock()
    trace: list[tuple[int, dict]] = []

    def on_low(msg):
        row = low_row(msg)
        with lock:
            trace.append((int(msg.tick), row))

    ChannelFactoryInitialize(args.domain, args.interface)
    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(on_low, 0)
    print(f'recording rt/lowstate for {args.duration:.0f}s (subscriber only, nothing sent)',
          flush=True)
    time.sleep(args.duration)
    sub.Close()

    with lock:
        recorded = list(trace)
    if len(recorded) < 100:
        print(f'NO DATA: only {len(recorded)} samples', file=sys.stderr)
        return 2
    span = (recorded[-1][0]-recorded[0][0])*.001
    print(f'recorded {len(recorded)} samples over {span:.1f}s '
          f'({len(recorded)/span:.0f} Hz)\n', flush=True)

    print(f'{"rate":>8} {"steps":>7} {"cost/s":>8} {"end xy":>9} {"end z":>9} '
          f'{"drift/s":>9} {"feet p50":>9} {"valid":>7}')
    reference = None
    for text in args.rates.split(','):
        rate = float(text)
        leg = LegPose(calibration_seconds=0, rate_hz=rate)
        feet, valid, steps = [], 0, 0
        started = time.perf_counter()
        for tick, row in recorded:
            pose = leg.update(row, tick)
            if pose is None:
                continue
            steps += 1
            feet.append(pose['reliable_feet'])
            valid += bool(pose['pose_valid'])
        elapsed = time.perf_counter()-started
        position = leg.estimator.pos
        xy = float(np.linalg.norm(position[:2]))
        label = 'every' if rate == 0 else f'{rate:.0f} Hz'
        # Cost per second of robot time: what the callback thread actually has to spend.
        print(f'{label:>8} {steps:7d} {elapsed/span*1e3:7.1f}ms {xy*100:8.2f}cm '
              f'{position[2]*100:8.2f}cm {xy/span*100:8.2f}cm {int(np.median(feet)):9d} '
              f'{valid/max(1,steps)*100:6.1f}%')
        if reference is None:
            reference = position.copy()
        else:
            delta = np.linalg.norm((position-reference)[:2])
            print(f'{"":>8} vs every-sample: xy {delta*100:.2f} cm '
                  f'({delta/span*100:.2f} cm/s)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
