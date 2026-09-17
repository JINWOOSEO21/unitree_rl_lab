"""What does one LowState sample cost on this CPU? No DDS, no robot -- pure offline timing.

The bridge runs every stage below inside the 500 Hz DDS callback, holding the GIL. If the
total exceeds 2 ms the callback cannot keep up with its own topic, and everything else in
the process -- the cloud reader, the tick loop, the GPU call -- starves behind it. Feed a
canned LowState row through each stage and report the per-call cost and the duty it implies.

Usage: python3 low_cost.py --parkour <dir> [--rate 500] [--n 2000]
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--parkour', required=True)
    ap.add_argument('--rate', type=float, default=500.0, help='LowState Hz the stage must sustain')
    ap.add_argument('--n', type=int, default=2000)
    args = ap.parse_args()

    sys.path.insert(0, args.parkour)
    sys.path.insert(0, args.parkour + '/tools')
    import numpy as np
    import yaml
    from go2_leg_pose import LegPose, ROOT
    from policy_input_guard import GuardConfig, validate_lowstate
    from em_sidecar.kinematics import Go2Kinematics, quat_to_mat

    contract = yaml.safe_load((ROOT/'contract/deploy.yaml').read_text())
    row = {'imu_state': {'quaternion': [1., 0., 0., 0.], 'gyroscope': [0., 0., 0.]},
           'motor_state': [{'q': q, 'dq': 0.} for q in contract['default_joint_pos']['sdk']]
                          + [{'q': 0., 'dq': 0.} for _ in range(8)],
           'foot_force': [100., 100., 100., 100.]}
    guard = GuardConfig()
    kin = Go2Kinematics(ROOT/'contract/em_geometry.npz')
    q_il = np.asarray([m['q'] for m in row['motor_state'][:12]])
    quat = np.asarray(row['imu_state']['quaternion'])

    leg = LegPose(calibration_seconds=0)
    tick = [1000]

    def leg_update():
        tick[0] += 2
        leg.update(row, tick[0])

    stages = [
        ('validate_lowstate', lambda: validate_lowstate(row, guard)),
        ('quat_to_mat', lambda: quat_to_mat(quat)),
        ('kin.link_poses_base (FK)', lambda: kin.link_poses_base(q_il)),
        ('LegPose.update (whole callback core)', leg_update),
    ]

    budget_ms = 1000.0/args.rate
    print(f'per-call cost at {args.rate:.0f} Hz (budget {budget_ms:.2f} ms/sample), '
          f'n={args.n}', flush=True)
    for name, function in stages:
        for _ in range(50):  # warm the caches and any lazy numpy dispatch
            function()
        samples = []
        for _ in range(args.n):
            started = time.perf_counter()
            function()
            samples.append((time.perf_counter()-started)*1e3)
        samples.sort()
        p50 = samples[len(samples)//2]
        duty = p50/budget_ms*100.0
        flag = '  <-- OVER BUDGET' if duty >= 100 else ''
        print(f'  {name:38s} p50={p50:7.3f} p90={samples[int(len(samples)*.9)]:7.3f} '
              f'max={samples[-1]:7.3f} ms   mean={statistics.fmean(samples):6.3f}   '
              f'{duty:6.1f}% of one core{flag}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
