"""Go2 receive-and-infer-only runner: offline replay or a subscriber-only bridge.

The bridge runs in the existing GPU/DDS environment; ONNX runs in this process.
IPC is a local JSONL pipe. No command publisher or robot service client exists.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

from policy_input_guard import GuardConfig, PolicyInputGuard
from replay_go2_policy import ROOT, History, ActionDelay, build_prop, euler_xyz_from_quat, wrap_to_pi


class ShadowPolicy:
    def __init__(self, contract, runtime, session, guard_config=None):
        self.contract, self.runtime, self.session = contract, runtime, session
        self.guard = PolicyInputGuard(guard_config)
        self.lock = threading.Lock()
        self.fatal = None
        self.rejections = Counter()
        self.history = History()
        a = contract['action']
        self.default = np.asarray(contract['default_joint_pos']['isaaclab'], dtype=np.float32)
        self.mapping = np.asarray(contract['index_maps']['il_to_sdk'], dtype=np.int64)
        self.feet = np.asarray(contract['index_maps']['il_foot_to_sdk'], dtype=np.int64)
        if sorted(self.mapping.tolist()) != list(range(12)) or sorted(self.feet.tolist()) != list(range(4)):
            raise ValueError('invalid joint/foot mapping')
        delay = int(runtime.get('action_delay_override', -1))
        self.delay = ActionDelay(delay if delay >= 0 else int(a['delay_steps']),
                                 tuple(a['clip']), a['scale'], self.default)
        self.previous_contact = np.zeros(4, dtype=bool)
        self.target_yaw = None
        self.reset_count = 0

    def offer(self, event):
        with self.lock:
            kind = event['kind']
            if kind == 'fatal':
                self.fatal = self.fatal or event['reason']
                return
            if kind == 'fault':
                self.rejections['bridge_fault: '+event['reason']] += 1
                self.guard.reset_stream()
                return
            kwargs = {k: event[k] for k in ('receipt_ns','source_ns','source_id')}
            if kind == 'low':
                result = self.guard.offer_low(event['low'], **kwargs)
            elif kind == 'scan':
                result = self.guard.offer_scan(event['scan'], **kwargs)
            else:
                raise ValueError('unknown bridge event')
            if not result.accepted:
                self.rejections[result.reason] += 1
                if result.restart_required:
                    self.fatal = result.reason

    def step(self, now_ns, *, live=False):
        with self.lock:
            if live:
                now_ns = time.monotonic_ns()
            decision = self.guard.evaluate(receipt_now_ns=now_ns)
            if self.fatal or not decision.allowed:
                return {'tick_ns':now_ns, 'allowed':False,
                        'reasons':[self.fatal] if self.fatal else list(decision.reasons)}
            if decision.reset_history:
                self.history.reset()
                self.delay.reset()
                self.previous_contact[:] = False
                self.target_yaw = None
                self.reset_count += 1
                self.guard.acknowledge_history_reset()
        _, _, yaw = euler_xyz_from_quat(decision.low['imu_state']['quaternion'])
        if self.target_yaw is None:
            self.target_yaw = yaw
        prop, self.previous_contact = build_prop(
            decision.low, self.default, self.mapping, self.feet, self.delay.last_raw(),
            self.previous_contact, float(self.runtime['contact_threshold']),
            float(self.runtime['cmd_vx_min']), float(np.float32(1.5)*wrap_to_pi(self.target_yaw-yaw)))
        if not self.history.frames:
            self.history.prime(prop)
        history = self.history.value()
        if not np.isfinite(prop).all() or not np.isfinite(history).all():
            self.fatal = 'nonfinite assembled observation'
            return {'tick_ns':now_ns,'allowed':False,'reasons':[self.fatal]}
        started = time.perf_counter_ns()
        try:
            raw = np.asarray(self.session.run(None, {'prop':prop[None], 'hist':history[None],
                                                     'scan':decision.scan[None]})[0],dtype=np.float32)
            if raw.shape != (1,12) or not np.isfinite(raw).all():
                raise ValueError('invalid ONNX action')
            raw = raw[0]
            target = self.delay.push(raw)
            if not np.isfinite(target).all():
                raise ValueError('invalid target')
        except Exception as exc:
            self.fatal = f'inference failed: {exc}'
            return {'tick_ns':now_ns,'allowed':False,'reasons':[self.fatal]}
        inference_ms = (time.perf_counter_ns()-started)*1e-6
        # Check the ages of the actual inference inputs, not newer replacements.
        c = self.guard.config
        with self.lock:
            finished_ns = time.monotonic_ns() if live else now_ns
            stale = (finished_ns-decision.low_provenance.source_ns > c.low_source_timeout_ns or
                     finished_ns-decision.scan_provenance.source_ns > c.scan_source_timeout_ns)
            current = self.guard.evaluate(receipt_now_ns=finished_ns)
            invalidated = self.fatal or not current.allowed or current.reset_history
            if stale or invalidated:
                self.guard.reset_stream()
                return {'tick_ns':now_ns,'allowed':False,'reasons':['inputs invalidated during inference'],
                        'inference_ms':inference_ms}
        self.history.push(prop)
        sdk_target = np.empty(12, dtype=np.float32)
        sdk_target[self.mapping] = target
        return {'tick_ns':now_ns,'allowed':True, 'inference_ms':inference_ms,
                'input_low_age_ms':decision.low_source_age_ns*1e-6,
                'input_scan_source_age_ms':decision.scan_source_age_ns*1e-6,
                'raw_action':raw.tolist(),'q_target_il':target.tolist(),'q_target_sdk':sdk_target.tolist()}


def offline_events(recording, scan_path):
    lows = [json.loads(line) for line in (recording/'lowstate.jsonl').read_text().splitlines()]
    events = [{'kind':'low','receipt_ns':r['steady_ns'],'source_ns':r['steady_ns'],
               'source_id':r['tick'],'low':r} for r in lows]
    clouds = [json.loads(line) for line in (recording/'utlidar_cloud_base/cloud.jsonl').read_text().splitlines()]
    with np.load(scan_path) as scan:
        for i in np.flatnonzero(scan['accepted']):
            ci = int(scan['source_cloud_index'][i])
            stamp = clouds[ci]['stamp']
            events.append({'kind':'scan','receipt_ns':int(scan['tick_steady_ns'][i]),
                           'source_ns':int(scan['source_cloud_steady_ns'][i]),
                           'source_id':int(stamp['sec'])*1_000_000_000+int(stamp['nanosec']),
                           'scan':scan['scan'][i].tolist()})
        start = int(scan['tick_steady_ns'][0])
    events.sort(key=lambda e:e['receipt_ns'])
    return events, start, int(lows[-1]['steady_ns'])


def replay(core, recording, scan_path):
    events, start, end = offline_events(recording, scan_path)
    index = 0
    for tick in range(start,end+1,20_000_000):
        while index < len(events) and events[index]['receipt_ns'] <= tick:
            core.offer(events[index]); index += 1
        yield core.step(tick)


def live(core, args):
    command = [str(args.bridge_python),str(Path(__file__).with_name('go2_sensor_bridge.py')),
               '--interface',args.interface,'--domain',str(args.domain),
               '--duration',str(args.duration),'--emcupy-root',str(args.emcupy_root),
               '--odom',args.odom,'--leg-contact-threshold',str(args.leg_contact_threshold)]
    child = subprocess.Popen(command,stdout=subprocess.PIPE,text=True,bufsize=1)
    eof = threading.Event()
    stopping = threading.Event()
    def drain():
        try:
            for line in child.stdout:
                core.offer(json.loads(line))
        except Exception as exc:
            core.offer({'kind':'fatal','reason':f'bridge protocol failure: {exc}'})
        finally:
            if not stopping.is_set():
                core.offer({'kind':'fatal','reason':'sensor bridge ended unexpectedly'})
            eof.set()
    reader = threading.Thread(target=drain,daemon=True)
    reader.start()
    start = time.monotonic_ns()
    tick = start
    try:
        while time.monotonic_ns()-start < args.duration*1e9:
            time.sleep(max(0,(tick-time.monotonic_ns())*1e-9))
            now = time.monotonic_ns()
            yield core.step(now,live=True)
            if eof.is_set() or core.fatal:
                break
            tick = max(tick+20_000_000,time.monotonic_ns())
    finally:
        stopping.set()
        if child.poll() is None:
            child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill(); child.wait(timeout=5)
        reader.join(timeout=2)
        child.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--recording',type=Path)
    mode.add_argument('--interface')
    parser.add_argument('--scan-replay',type=Path,default=ROOT/'captures/cloud_base_replay/replay.npz')
    parser.add_argument('--contract',type=Path,default=ROOT/'contract/deploy.yaml')
    parser.add_argument('--runtime-config',type=Path,default=ROOT.parent/'robots/go2/config/config.yaml')
    parser.add_argument('--policy',type=Path,default=ROOT/'contract/policy.onnx')
    parser.add_argument('--bridge-python',type=Path,default=Path('/home/seo-jinwoo/miniconda3/envs/env_isaaclab/bin/python'))
    parser.add_argument('--emcupy-root',type=Path,default=Path('/home/seo-jinwoo/workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy'))
    parser.add_argument('--domain',type=int,default=0)
    parser.add_argument('--odom',choices=('leg','robot'),default='leg',
                        help='Live bridge pose source; offline replay uses its saved scan')
    parser.add_argument('--leg-contact-threshold',type=float,default=20.0)
    parser.add_argument('--duration',type=float,default=30)
    parser.add_argument('--output-dir',type=Path,required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 300 or not 0 <= args.domain <= 232:
        parser.error('duration in (0,300], domain in [0,232]')
    # Reject accidental overwrite of diagnostic results.
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if any((args.output_dir/name).exists() for name in ('inference.jsonl','summary.json')):
        parser.error('output directory already has inference results; choose a fresh directory')
    import onnxruntime as ort
    contract = yaml.safe_load(args.contract.read_text())
    runtime = yaml.safe_load(args.runtime_config.read_text())['FSM']['Parkour']
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(args.policy),sess_options=options,providers=['CPUExecutionProvider'])
    core = ShadowPolicy(contract,runtime,session)
    results = []
    with (args.output_dir/'inference.jsonl').open('w') as output:
        stream = replay(core,args.recording,args.scan_replay) if args.recording else live(core,args)
        for result in stream:
            output.write(json.dumps(result,allow_nan=False)+'\n')
            results.append(result)
    good = [r for r in results if r['allowed']]
    times = [r['inference_ms'] for r in good]
    reasons = Counter(reason for r in results if not r['allowed'] for reason in r['reasons'])
    summary = {'mode':'offline' if args.recording else 'live_receive_only',
               'odometry_source':'saved_scan' if args.recording else args.odom,
               'scheduled_ticks':len(results),'inferred_ticks':len(good),'blocked_ticks':len(results)-len(good),
               'blocked_reasons':dict(reasons),'rejected_events':dict(core.rejections),
               'history_initializations':core.reset_count,'fatal':core.fatal,
               'guard_config':asdict(core.guard.config),
               'inference_ms':{'median':float(np.median(times)),'max':float(max(times))} if times else None,
               'publish_commands':False,'motor_ready':False,
               'limitations':['Static open-loop capture does not validate gait stability.',
                              'Offline map availability is synthetic; live GPU/DDS scheduling must be measured.',
                              'LowState source_ns is PC receive time; tick checks detect duplicates/restarts but not sensor clock latency.',
                              '35mm height discrepancy and contact calibration remain unresolved.',
                              'Guards are in this receive-only runner, not the existing motor controller.']}
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    if core.fatal:
        raise SystemExit(1)


if __name__=='__main__': main()
