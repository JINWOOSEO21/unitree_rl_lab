"""Go2 raw-cloud/odom/LowState bridge; stdout is local JSONL by default.

Optional --publish-scandots sends terrain only; no motor command or service RPC.
Map processing uses
the measured hardware LiDAR -> base_link transform and the offline map backend.
Mapping uses leg odometry by default; --odom mit selects body IMU/leg Kalman fusion.
--odom robot selects robot_odom. MIT needs a stationary 10 s gyro calibration.
Missing/stale/unsupported poses block scan output.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import redirect_stdout, ExitStack
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np

from replay_go2_base_scan import (
    EXPECTED_FRAME, PARKOUR_ROOT, _load_backend, backend_input_from_base_cloud, odom_pose,
    validate_policy_scan, Go2Kinematics, quat_to_mat, yaw_from_quat,
)
from em_sidecar.go2_cloud import RAW_FRAME, raw_cloud_to_base
from go2_leg_pose import LegPose
from go2_mit_pose import MitPose
from go2_gyro_bias_output import GyroBiasOutput
from policy_input_guard import GuardConfig, validate_lowstate
from go2_scandots_output import ScandotsOutput


def low_row(msg, include_acceleration=False):
    row = {
        'imu_state': {'quaternion': list(msg.imu_state.quaternion),
                      'gyroscope': list(msg.imu_state.gyroscope)},
        'motor_state': [{'q': m.q, 'dq': m.dq} for m in msg.motor_state],
        'foot_force': list(msg.foot_force),
    }
    if include_acceleration:
        row['imu_state']['accelerometer'] = list(msg.imu_state.accelerometer)
    return row


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
                raise ValueError('mit_odom_no_reliable_support' if row.get('source') == 'mit' else 'leg_odom_no_reliable_support')
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

    def warmup(self):
        """Pay the backend's one-time CUDA/CuPy JIT before any subscriber exists.

        The first update() compiles the map kernels: 207 ms measured on the Jetson Orin NX
        against a 31 ms steady-state median. Left until the tick loop, that call holds the
        GIL long enough to starve the DDS reader thread and open a >100 ms hole in
        rt/lowstate, which the leg estimator then reports as a gap. Run it here on a
        synthetic flat cloud, while there is no reader thread to starve, and throw the
        result away so the live map still starts empty.

        Warmup is an optimisation, never a precondition. The scan it produces is meaningless,
        so a failure to validate it must not stop the bridge from starting; report it on
        stderr and carry on with cold kernels.
        """
        rng = np.random.default_rng(0)
        xy = rng.uniform(-1.5, 1.5, size=(4096, 2))
        points = np.column_stack([xy, np.full(len(xy), -0.30)]).astype(np.float32)
        row = {'frame_id': 'odom', 'child_frame_id': EXPECTED_FRAME,
               'position': {'x': 0.0, 'y': 0.0, 'z': 0.30},
               'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}}
        started = time.perf_counter()
        error = None
        try:
            self.update(None, row, base_points=points)
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        try:
            self.torch.cuda.synchronize()
        finally:
            self.backend.clear([0])
            self.last_position = None  # A synthetic pose must not gate the first live update.
        print(f'map warmup {(time.perf_counter()-started)*1e3:.0f} ms'
              + (f' (kernels may still be cold: {error})' if error else ''))

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


def dependencies(odom_source="leg"):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    import torch
    import cupy
    if odom_source == 'mit':
        # Defer participant creation until after interface-specific initialization.
        from go2_mit_dds import subscriber_type
        factory = [None]
        def initialize(domain, interface):
            ChannelFactoryInitialize(domain, interface)
            factory[0] = subscriber_type(domain)
        def subscribe(name, typ):
            return factory[0](name, typ)
        return initialize, subscribe, LowState_, PointCloud2_, Odometry_
    return ChannelFactoryInitialize, ChannelSubscriber, LowState_, PointCloud2_, Odometry_


def run(interface, domain, duration, emcupy_root, emit, odom_source='leg', leg_contact_threshold=20.0,
        publish_scandots=False, scandots_topic='rt/parkour/scandots', leg_odom_hz=100.0, mit_odom_hz=75.0):
    if odom_source not in ('leg', 'robot', 'mit'):
        raise ValueError('odom source must be leg, mit or robot')
    initialize, subscriber, low_type, cloud_type, odom_type = dependencies(odom_source)
    mapper = BaseMapper(emcupy_root)
    # Before any subscriber exists, so the JIT cannot starve the DDS reader. See warmup().
    mapper.warmup()
    # The estimator runs slower than the topic on purpose: its kinematics cost 1.5 ms per
    # sample on the Jetson, and at 500 Hz that holds the GIL for 76 % of a core -- the cloud
    # reader, the tick loop and the GPU call all starve behind it. The map consumes pose at
    # 10 Hz and preceding_pose accepts one up to 20 ms old, so 100 Hz leaves margin on both.
    estimator_type = MitPose if odom_source == 'mit' else LegPose
    estimator_hz = mit_odom_hz if odom_source == 'mit' else leg_odom_hz
    leg = (estimator_type(contact_threshold=leg_contact_threshold, rate_hz=estimator_hz)
           if odom_source in ('leg', 'mit') else None)
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
    rebuild_map = [False]  # set by the reader thread, acted on by the tick loop (GPU work)

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
            low = low_row(msg, include_acceleration=odom_source == 'mit')
            validate_lowstate(low, GuardConfig())
            pose = None
            gap_s = None
            with lock:
                if last_low[1] is not None and int(msg.tick) < last_low[1]:
                    raise RuntimeError('LowState source clock regressed; restart required')
                if int(msg.tick) != last_low[1]:
                    last_low[:] = [now, int(msg.tick)]
                if leg is not None:
                    pose = leg.update(low, msg.tick)
                    if pose is not None:
                        poses.append((now, pose))
                        gap_s = pose['pose_gap_s']
                        if gap_s is not None or pose.get('map_reset_required', False):
                            rebuild_map[0] = True
                        if last_support[0] and not pose['pose_valid']:
                            generation[0] += 1
                        last_support[0] = pose['pose_valid']
                        if not pose['pose_valid'] and output is not None:
                            output.invalidate()
            event = {'kind': 'low', 'receipt_ns': now, 'source_ns': now,
                     'source_id': int(msg.tick), 'low': low}
            if pose is not None:
                event['mit_odometry' if odom_source == 'mit' else 'leg_odometry'] = pose
            emit(event)
            if gap_s is not None:
                # Degraded, not fatal: bumps generation and invalidates the current output,
                # which is what a pose that skipped a stretch of travel deserves.
                failure('fault', f'lowstate gap {gap_s*1000:.0f} ms '
                                 f'(#{pose["pose_gaps"]}); rebuilding map from next cloud')
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
    runtime_stack = ExitStack()
    try:
        if odom_source == 'mit':
            from go2_mit_dds import initialized_heap_gc_scope
            runtime_stack.enter_context(initialized_heap_gc_scope())
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
            for sub in subs:
                if hasattr(sub, 'check'):
                    sub.check()
            loop_now = time.monotonic()
            next_tick = max(next_tick+0.1, loop_now)
            with lock:
                item = latest[0]
                pose_snapshot = list(poses)
                version = generation[0]
                rebuild = rebuild_map[0]
                rebuild_map[0] = False
            if rebuild:
                # Everything in the map predates a stretch of travel the pose never
                # integrated, so it is offset by that unmeasured distance. Drop it and let
                # the next cloud refill it, exactly as the cloud-fault path does.
                try:
                    mapper.discard_map()
                except Exception as exc:
                    failure('fatal', f'map reset failed: {exc}')
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
                map_row = row.get('mapping_pose', row)
                scan, valid, upper = mapper.update(cloud, map_row)
                with lock:
                    now = time.monotonic_ns()
                    if fatal.is_set() or generation[0] != version:
                        raise ValueError('inputs invalidated during map computation')
                    if now-receipt > 200_000_000:
                        raise ValueError('cloud_too_old_after_mapping')
                    if last_low[0] is None or now-last_low[0] > 20_000_000:
                        raise ValueError('lowstate_too_old_after_mapping')
                    if not poses or poses[-1][1].get('pose_valid') is False:
                        raise ValueError('mit_odom_no_reliable_support' if odom_source == 'mit' else 'leg_odom_no_reliable_support')
                    if output is not None:
                        position, _ = odom_pose(map_row)
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
                try:
                    if bias_output is not None:
                        bias_output.close(0 if last_low[1] is None else last_low[1])
                finally:
                    runtime_stack.close()


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
    parser.add_argument('--odom', choices=('leg','robot','mit'), default='leg')
    parser.add_argument('--leg-contact-threshold', type=float, default=20.0,
                        help='Raw foot_force threshold; hardware calibration remains pending')
    parser.add_argument('--leg-odom-hz', type=float, default=100.0,
                        help='Leg estimator rate; 0 runs it on every LowState sample (500 Hz)')
    parser.add_argument('--mit-odom-hz', type=float, default=75.0,
                        help='MIT estimator rate; default 75 Hz leaves margin for mapping')
    parser.add_argument('--check-dependencies', action='store_true')
    args = parser.parse_args()
    if args.check_dependencies:
        dependencies(args.odom)
        print('DDS types and torch/cupy import OK; no DDS participant created')
        return
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
                pose = event.get('mit_odometry', event.get('leg_odometry'))
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
                             leg_calibration=last_calibration[0], odom_source=args.odom)
            output.write(json.dumps(event, allow_nan=False, separators=(',', ':'))+'\n')
            output.flush()
    with redirect_stdout(sys.stderr):
        try:
            code = run(args.interface, args.domain, args.duration, args.emcupy_root, emit,
                       args.odom, args.leg_contact_threshold, args.publish_scandots,
                       args.scandots_topic, leg_odom_hz=args.leg_odom_hz, mit_odom_hz=args.mit_odom_hz)
        except KeyboardInterrupt:
            code = 0
        except Exception as exc:
            emit({'kind': 'fatal', 'reason': str(exc)})
            code = 1
    raise SystemExit(code)


if __name__ == '__main__':
    main()
