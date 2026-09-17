"""Go2 raw-cloud/odom/LowState bridge; stdout is local JSONL by default.

Optional --publish-scandots sends terrain only; no motor command or service RPC.
Map processing uses
the measured hardware LiDAR -> base_link transform and the offline map backend.
Mapping uses leg odometry by default, or robot_odom with --odom robot.
Missing/stale/unsupported poses block scan output.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np

from replay_go2_base_scan import (
    PARKOUR_ROOT, _load_backend, backend_input_from_base_cloud, odom_pose,
    validate_policy_scan, Go2Kinematics, quat_to_mat, yaw_from_quat,
)
from em_sidecar.go2_cloud import RAW_FRAME, raw_cloud_to_base
from go2_leg_pose import LegPose
from go2_gyro_bias_output import GyroBiasOutput
from policy_input_guard import GuardConfig, validate_lowstate
from go2_scandots_output import ScandotsOutput


def low_row(msg):
    return {
        'imu_state': {'quaternion': list(msg.imu_state.quaternion),
                      'gyroscope': list(msg.imu_state.gyroscope)},
        'motor_state': [{'q': m.q, 'dq': m.dq} for m in msg.motor_state],
        'foot_force': list(msg.foot_force),
    }


def pose_row(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return {'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation': {'w': q.w, 'x': q.x, 'y': q.y, 'z': q.z}}


def stamp_id(msg):
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid sensor header timestamp')
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def preceding_pose(poses, cloud_ns, max_age_ns=20_000_000):
    for receipt, row in reversed(poses):
        if receipt <= cloud_ns:
            if cloud_ns - receipt > max_age_ns:
                raise ValueError('odom_too_old')
            if row.get('pose_valid') is False:
                raise ValueError('leg_odom_no_reliable_support')
            odom_pose(row)  # strict frame and finite-pose validation
            return row
    raise ValueError('missing_preceding_odom')


def gyro_bias_status(leg, low_receipt_ns, low_tick, now_ns):
    """Return one heartbeat state without changing the estimator."""
    fresh = (low_receipt_ns is not None and
             0 <= now_ns-low_receipt_ns <= 100_000_000)
    return (bool(leg.calibrated and fresh), leg.gyro_bias.copy(),
            0 if low_tick is None else int(low_tick))


class BaseMapper:
    def __init__(self, emcupy_root, device='cuda:0'):
        import torch
        self.torch, self.device = torch, device
        self.backend = _load_backend(emcupy_root, device)
        self.xy = Go2Kinematics(PARKOUR_ROOT / 'contract/em_geometry.npz').scan_offsets_xy
        self.last_position = None

    def discard_map(self):
        # A rejected update may already have mutated the persistent GPU map.
        # Keep the last pose so clearing cannot bypass the discontinuity check.
        self.backend.clear([0])

    def update(self, cloud, row, base_points=None):
        points = raw_cloud_to_base(cloud) if base_points is None else base_points
        position, quat = odom_pose(row)
        if self.last_position is not None and np.linalg.norm(position-self.last_position) > 1.0:
            raise RuntimeError('odometry discontinuity: restart bridge to reset map')
        if not len(points):
            raise ValueError('empty_finite_cloud')
        rotation = quat_to_mat(quat)
        pts, rot, pos = backend_input_from_base_cloud(points, position, rotation)
        def tensor(x):
            return self.torch.as_tensor(x, dtype=self.torch.float32, device=self.device)
        self.backend.update([tensor(pts)], tensor(rot)[None], tensor(pos)[None],
                            tensor(position)[None], tensor(rotation)[None])
        yaw = yaw_from_quat(quat)
        c, s = np.cos(yaw), np.sin(yaw)
        xy = np.stack([position[0]+c*self.xy[:,0]-s*self.xy[:,1],
                       position[1]+s*self.xy[:,0]+c*self.xy[:,1]], axis=-1)
        scan, valid, upper = self.backend.sample(tensor(xy)[None], tensor([position[2]]))
        self.last_position = position
        return (validate_policy_scan(scan[0].detach().cpu().numpy()),
                valid[0].detach().cpu().numpy(), upper[0].detach().cpu().numpy())


def dependencies():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    import torch
    import cupy
    return ChannelFactoryInitialize, ChannelSubscriber, LowState_, PointCloud2_, Odometry_


def run(interface, domain, duration, emcupy_root, emit, odom_source='leg', leg_contact_threshold=20.0,
        publish_scandots=False, scandots_topic='rt/parkour/scandots'):
    if odom_source not in ('leg', 'robot'):
        raise ValueError('odom source must be leg or robot')
    initialize, subscriber, low_type, cloud_type, odom_type = dependencies()
    mapper = BaseMapper(emcupy_root)
    leg = LegPose(contact_threshold=leg_contact_threshold) if odom_source == 'leg' else None
    lock = threading.RLock()
    poses = deque(maxlen=2000)
    latest = [None]
    last_stamp = {'cloud': None, 'odom': None}
    fatal = threading.Event()
    output = None
    bias_output = None
    generation = [0]
    last_low = [None, None]  # receipt, unique tick
    last_support = [False]

    def failure(kind, reason):
        with lock:
            generation[0] += 1
            if kind == 'fatal':
                fatal.set()
            if output is not None:
                try:
                    output.invalidate()
                except Exception as exc:
                    kind, reason = 'fatal', f'{reason}; {exc}'
                    fatal.set()
            emit({'kind': kind, 'reason': reason})

    def low_callback(msg):
        now = time.monotonic_ns()
        try:
            low = low_row(msg)
            validate_lowstate(low, GuardConfig())
            pose = None
            with lock:
                if last_low[1] is not None and int(msg.tick) < last_low[1]:
                    raise RuntimeError('LowState source clock regressed; restart required')
                if int(msg.tick) != last_low[1]:
                    last_low[:] = [now, int(msg.tick)]
                if leg is not None:
                    pose = leg.update(low, msg.tick)
                    if pose is not None:
                        poses.append((now, pose))
                        if last_support[0] and not pose['pose_valid']:
                            generation[0] += 1
                        last_support[0] = pose['pose_valid']
                        if not pose['pose_valid'] and output is not None:
                            output.invalidate()
            event = {'kind': 'low', 'receipt_ns': now, 'source_ns': now,
                     'source_id': int(msg.tick), 'low': low}
            if pose is not None:
                event['leg_odometry'] = pose
            emit(event)
        except Exception as exc:
            failure('fatal' if leg is not None else 'fault', f'low: {exc}')

    def sensor_callback(kind, msg):
        now = time.monotonic_ns()
        try:
            stamp = stamp_id(msg)
            with lock:
                previous = last_stamp[kind]
                if previous is not None and stamp <= previous:
                    if stamp < previous:
                        failure('fatal', f'{kind} source clock regressed; restart required')
                    # Duplicate messages do not replace the original receive time.
                    return
                last_stamp[kind] = stamp
                if kind == 'odom':
                    row = pose_row(msg)
                    odom_pose(row)
                    poses.append((now, row))
                else:
                    if msg.header.frame_id != RAW_FRAME:
                        raise ValueError(f'raw cloud must be {RAW_FRAME}')
                    latest[0] = (now, stamp, msg)
        except Exception as exc:
            failure('fault', f'{kind}: {exc}')

    initialize(domain, interface)
    subs = []
    try:
        if publish_scandots:
            output = ScandotsOutput(scandots_topic)
            if leg is not None:
                bias_output = GyroBiasOutput()
        channels = [
            ('rt/lowstate', low_type, low_callback),
            ('rt/utlidar/cloud', cloud_type, lambda m: sensor_callback('cloud', m)),
        ]
        if odom_source == 'robot':
            channels.append(('rt/utlidar/robot_odom', odom_type, lambda m: sensor_callback('odom', m)))
        for topic, typ, callback in channels:
            sub = subscriber(topic, typ)
            sub.Init(callback, 0)  # sample time at DDS callback, without queued delivery
            subs.append(sub)
        start = time.monotonic()
        next_tick = start
        next_bias = start + 0.2
        last_cloud = None
        while (duration == 0 or time.monotonic()-start < duration) and not fatal.is_set():
            time.sleep(max(0, next_tick-time.monotonic()))
            loop_now = time.monotonic()
            next_tick = max(next_tick+0.1, loop_now)
            with lock:
                item = latest[0]
                pose_snapshot = list(poses)
                version = generation[0]
            if bias_output is not None and loop_now >= next_bias:
                try:
                    with lock:
                        now_ns = time.monotonic_ns()
                        calibrated, bias, source_tick = gyro_bias_status(
                            leg, last_low[0], last_low[1], now_ns)
                    bias_output.publish(calibrated, bias, source_tick)
                except Exception as exc:
                    failure('fatal', f'gyro bias: {exc}')
                next_bias = max(next_bias + 0.2, loop_now)
            if item is None or item[1] == last_cloud:
                if output is not None:
                    with lock:
                        now = time.monotonic_ns()
                        if (item is None or now-item[0] > 200_000_000 or
                                last_low[0] is None or now-last_low[0] > 20_000_000):
                            output.invalidate()
                continue
            receipt, stamp, cloud = item
            last_cloud = stamp
            map_update_started = False
            try:
                if time.monotonic_ns()-receipt > 200_000_000:
                    raise ValueError('cloud_too_old')
                row = preceding_pose(pose_snapshot, receipt)
                map_update_started = True
                scan, valid, upper = mapper.update(cloud, row)
                with lock:
                    now = time.monotonic_ns()
                    if fatal.is_set() or generation[0] != version:
                        raise ValueError('inputs invalidated during map computation')
                    if now-receipt > 200_000_000:
                        raise ValueError('cloud_too_old_after_mapping')
                    if last_low[0] is None or now-last_low[0] > 20_000_000:
                        raise ValueError('lowstate_too_old_after_mapping')
                    if not poses or poses[-1][1].get('pose_valid') is False:
                        raise ValueError('leg_odom_no_reliable_support')
                    if output is not None:
                        position, _ = odom_pose(row)
                        output.publish(scan, position, receipt, now)
                    emit({'kind': 'scan', 'receipt_ns': now,
                          'source_ns': receipt, 'source_id': stamp, 'scan': scan.tolist(),
                          'odometry': row,
                          'valid_fraction': valid.tolist(), 'upper_bound_fraction': upper.tolist()})
            except ValueError as exc:
                failure('fault', str(exc))
                if map_update_started:
                    try:
                        mapper.discard_map()
                    except Exception as reset_error:
                        failure('fatal', f'map reset failed: {reset_error}')
            except Exception as exc:
                failure('fatal', str(exc))
        return 1 if fatal.is_set() else 0
    finally:
        try:
            for sub in subs:
                sub.Close()
        finally:
            try:
                if output is not None:
                    output.close()
            finally:
                if bias_output is not None:
                    bias_output.close(0 if last_low[1] is None else last_low[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--duration', type=float, default=0, help='Seconds; default 0 runs until Ctrl+C')
    parser.add_argument('--emcupy-root', type=Path,
                        default=Path.home()/'workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy')
    parser.add_argument('--publish-scandots', action='store_true', help='Publish terrain DDS for go2_ctrl')
    parser.add_argument('--scandots-topic', default='rt/parkour/scandots')
    parser.add_argument('--summary-only', action='store_true', help='Print counters once per second instead of all LowState rows')
    parser.add_argument('--odom', choices=('leg','robot','lio'), default='leg')
    parser.add_argument('--leg-contact-threshold', type=float, default=20.0,
                        help='Raw foot_force threshold; hardware calibration remains pending')
    parser.add_argument('--check-dependencies', action='store_true')
    args = parser.parse_args()
    if args.check_dependencies:
        dependencies()
        print('DDS types and torch/cupy import OK; no DDS participant created')
        return
    if args.odom == 'lio' and args.publish_scandots and args.scandots_topic != 'rt/parkour/scandots_lio_eval':
        parser.error('LIO is diagnostic: set --scandots-topic rt/parkour/scandots_lio_eval')
    if not args.interface or not 0 <= args.duration <= 300 or not 0 <= args.domain <= 232:
        parser.error('interface required; duration in [0,300], domain in [0,232]')
    output = sys.stdout
    write_lock = threading.Lock()
    counters = {'low':0, 'scan':0, 'fault':0, 'fatal':0}
    last_report = [0.0]
    last_reason = [None]
    last_calibration = [None]
    def emit(event):
        with write_lock:
            if args.summary_only:
                counters[event['kind']] += 1
                pose = event.get('leg_odometry')
                if pose is not None:
                    last_calibration[0] = {key: pose[key] for key in
                        ('gyro_calibrated', 'gyro_bias_rad_s', 'gyro_calibration_elapsed_s')
                        if key in pose}
                if event.get('reason'):
                    last_reason[0] = event['reason']
                now = time.monotonic()
                if event['kind'] != 'fatal' and now-last_report[0] < 1:
                    return
                last_report[0] = now
                event = dict(counters, last_event=event['kind'], last_fault=last_reason[0],
                             dds_scandots=args.publish_scandots,
                             leg_calibration=last_calibration[0])
            output.write(json.dumps(event, allow_nan=False, separators=(',', ':'))+'\n')
            output.flush()
    with redirect_stdout(sys.stderr):
        try:
            if args.odom == 'lio':
                from lio.map_shadow import run as run_lio
                code = run_lio(args.interface, args.domain, args.duration, args.emcupy_root, emit,
                               args.publish_scandots, args.scandots_topic)
            else:
                code = run(args.interface, args.domain, args.duration, args.emcupy_root, emit,
                           args.odom, args.leg_contact_threshold, args.publish_scandots, args.scandots_topic)
        except KeyboardInterrupt:
            code = 0
        except Exception as exc:
            emit({'kind': 'fatal', 'reason': str(exc)})
            code = 1
    raise SystemExit(code)


if __name__ == '__main__':
    main()
