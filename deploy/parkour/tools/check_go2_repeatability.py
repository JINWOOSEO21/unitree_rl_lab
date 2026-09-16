"""Offline repeatability and ONNX impact; serial kernel is diagnostic-only."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def ranges(v):
    d=np.ptp(v,axis=0)
    return {'max':float(d.max()),'mean':float(d.mean()),'p95':float(np.quantile(d,.95)),'p99':float(np.quantile(d,.99))}

def serial_add_points(backend):
    original=backend._add_points
    def ordered(*args,size):
        for i in range(size):
            current=list(args)
            current[6]=args[6][i:i+1]
            current[7]=args[7].reshape(-1)[i*3:(i+1)*3]
            original(*current,size=1)
    backend._add_points=ordered

def maps(args):
    from go2_sensor_bridge import BaseMapper,preceding_pose
    recording=args.recording
    clouds=[json.loads(x) for x in (recording/'utlidar_cloud_base/cloud.jsonl').read_text().splitlines()]
    poses=[json.loads(x) for x in (recording/'robot_odom.jsonl').read_text().splitlines()]
    poses=[(r['steady_ns'],r) for r in poses]
    with np.load(args.scan_replay) as data:
        schedule={k:data[k].copy() for k in ('tick_steady_ns','source_cloud_index','source_cloud_steady_ns','accepted')}
    accepted=np.flatnonzero(schedule['accepted'])
    def once(serial=False):
        mapper=BaseMapper(args.emcupy_root)
        if serial:serial_add_points(mapper.backend)
        scans=[];valid=[];upper=[]
        with (recording/'utlidar_cloud_base/cloud.bin').open('rb') as binary:
            for i in (accepted[:6] if serial else accepted):
                row=clouds[int(schedule['source_cloud_index'][i])]
                binary.seek(row['binary_offset'])
                cloud=NS(**{**row,'header':NS(frame_id=row['frame_id']),'data':binary.read(row['binary_size']),
                            'fields':[NS(**f) for f in row['fields']]})
                s,v,u=mapper.update(cloud,preceding_pose(poses,row['steady_ns']))
                scans.append(s);valid.append(v);upper.append(u)
        return np.asarray(scans),np.asarray(valid),np.asarray(upper)
    normal=[once() for _ in range(args.runs)]
    serial=[once(serial=True) for _ in range(3)]
    scan,valid,upper=(np.stack([r[i] for r in normal]) for i in range(3))
    serial_scan=np.stack([r[0] for r in serial])
    assert np.isfinite(scan).all() and np.max(np.abs(scan))<=1
    spread=np.ptp(scan,axis=0)
    full=np.all(valid>=.999999,axis=0)
    unknown=np.all((valid<=1e-6)&(upper<=1e-6),axis=0)
    groups={}
    for name,mask in [('fully_direct_all_runs',full),('mixed_or_upper_or_support_changed',~full&~unknown),('unknown_all_runs',unknown)]:
        groups[name]={'frame_cells':int(mask.sum()),'max_spread':float(spread[mask].max()) if mask.any() else 0,
                      'changed_over_1e_5':int((spread[mask]>1e-5).sum())}
    worst=np.unravel_index(np.argmax(spread),spread.shape)
    summary={'normal_runs':args.runs,'map_frames':len(accepted),'scan_repeat_range':ranges(scan),'source_groups':groups,
             'worst_frame_cell':[int(v) for v in worst],'worst_values':scan[:,worst[0],worst[1]].tolist(),
             'worst_direct_fractions':valid[:,worst[0],worst[1]].tolist(),'worst_upper_fractions':upper[:,worst[0],worst[1]].tolist(),
             'serialized_add_points':{'runs':3,'frames_per_run':6,'range':ranges(serial_scan),
                'parallel_same_six_frames_range':ranges(scan[:,:6]),'deployment_enabled':False},
             'backend_sha256':hashlib.sha256((ROOT/'vendored/elevation_map_backend.py').read_bytes()).hexdigest(),
             'scope':'recorded stationary cloud_base; no DDS; serialized kernel is diagnostic only'}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output_dir/'comparison.npz',scan_runs=scan,valid_runs=valid,upper_runs=upper,serial_scan=serial_scan,**schedule)
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

def policy(args):
    import yaml
    import onnxruntime as ort
    from replay_go2_policy import replay
    from audit_go2_targets import load_urdf_limits
    path=args.output_dir/'comparison.npz'
    with np.load(path) as data:arrays=dict(data)
    runtime=yaml.safe_load((ROOT.parent/'robots/go2/config/config.yaml').read_text())['FSM']['Parkour']
    outputs=[];isolated=[]
    session=ort.InferenceSession(str(ROOT/'contract/policy.onnx'),providers=['CPUExecutionProvider'])
    with tempfile.TemporaryDirectory(prefix='go2_repeatability_') as temp:
        for scans in arrays['scan_runs']:
            complete=np.full((len(arrays['accepted']),132),np.nan,dtype=np.float32)
            complete[arrays['accepted']]=scans
            scan_path=Path(temp)/'scan.npz'
            np.savez(scan_path,scan=complete,accepted=arrays['accepted'],tick_steady_ns=arrays['tick_steady_ns'])
            out,_=replay(args.recording/'lowstate.jsonl',scan_path,ROOT/'contract/deploy.yaml',ROOT/'contract/policy.onnx',
                         ROOT/'contract/golden_trace.npz',cmd_vx=runtime['cmd_vx_min'],action_delay_steps=runtime['action_delay_override'],
                         contact_threshold=runtime['contact_threshold'])
            outputs.append(out)
            reference=outputs[0]
            isolated.append(np.asarray([session.run(None,{'prop':p[None],'hist':h[None],'scan':s[None]})[0][0]
                                        for p,h,s in zip(reference['prop'],reference['hist'],out['scan'])]))
    actions=np.stack([r['raw_action'] for r in outputs]);targets=np.stack([r['q_target'] for r in outputs])
    assert np.isfinite(actions).all() and np.isfinite(targets).all()
    contract=yaml.safe_load((ROOT/'contract/deploy.yaml').read_text())
    names=contract['joint_names']['isaaclab']
    limits=load_urdf_limits(ROOT/'captures/frame_inspection_20260915/jetson_go2_description.urdf',names)
    lower=np.array([limits[n]['lower'] for n in names]);upper=np.array([limits[n]['upper'] for n in names])
    velocity=np.array([limits[n]['velocity'] for n in names])
    summary=json.loads((args.output_dir/'summary.json').read_text())
    summary['policy_effect']={'steps_per_run':len(actions[0]),'command_mps':runtime['cmd_vx_min'],
        'recurrent_raw_action_range':ranges(actions),'same_prop_history_action_range':ranges(np.stack(isolated)),
        'q_target_range_rad':ranges(targets),'q_target_max_range_deg':float(np.rad2deg(np.ptp(targets,axis=0).max())),
        'clip_exceeding_values':int((np.abs(actions)>4.8).sum()),
        'urdf_position_exceeding_values':int(((targets<lower)|(targets>upper)).sum()),
        'urdf_reference_rate_exceeding_values':int((np.abs(np.diff(targets,axis=1))/.02>velocity).sum()),
        'all_finite':True,'closed_loop_robot_validation':False}
    arrays.update(action_runs=actions,target_runs=targets,isolated_action_runs=np.stack(isolated),policy_tick_ns=outputs[0]['tick_steady_ns'])
    np.savez_compressed(path,**arrays)
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary['policy_effect'],indent=2))

def plot(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with np.load(args.output_dir/'comparison.npz') as d:
        scans=d['scan_runs'];actions=d['action_runs'];targets=d['target_runs'];ticks=d['policy_tick_ns']
    fig,axes=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
    im=axes[0].imshow(np.ptp(scans,axis=0).max(axis=0).reshape(11,12).T,origin='lower',aspect='auto')
    axes[0].set(xlabel='y-grid index',ylabel='x-grid index',title='Max scan range across runs/time')
    fig.colorbar(im,ax=axes[0],label='normalized height')
    t=(ticks-ticks[0])*1e-9
    axes[1].plot(t,np.ptp(actions,axis=0).max(axis=1))
    axes[1].set(xlabel='replay time [s]',ylabel='max joint action range',title='Policy recurrent sensitivity')
    axes[2].plot(t,np.rad2deg(np.ptp(targets,axis=0).max(axis=1)))
    axes[2].set(xlabel='replay time [s]',ylabel='max target range [deg]',title='Target differences; no motor commands')
    fig.savefig(args.output_dir/'impact.png',dpi=150);plt.close(fig)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=['maps','policy','plot'])
    p.add_argument('--recording',type=Path,default=ROOT/'captures/frame_inspection_20260915/dds_capture')
    p.add_argument('--scan-replay',type=Path,default=ROOT/'captures/cloud_base_replay/replay.npz')
    p.add_argument('--emcupy-root',type=Path,default=Path('/home/seo-jinwoo/workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy'))
    p.add_argument('--output-dir',type=Path,default=ROOT/'captures/repeatability')
    p.add_argument('--runs',type=int,default=6)
    args=p.parse_args()
    if args.runs<2:p.error('runs must be >=2')
    {'maps':maps,'policy':policy,'plot':plot}[args.phase](args)
if __name__=='__main__':main()
