"""Compare startup gyro calibration with an uncorrected saved LowState replay.

No DDS imports or robot connection. Uses only saved samples, not the full live
500 Hz stream. Compare both estimators over the same post-calibration interval.
"""
import argparse
import json

import numpy as np

from go2_leg_pose import LegPose


def replay(path):
    raw, corrected = LegPose(calibration_seconds=0), LegPose()
    start = None
    first_tick = None
    last_tick = None
    with open(path) as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'low':
                continue
            tick = event['source_id']
            a = raw.update(event['low'], tick)
            b = corrected.update(event['low'], tick)
            if b is None:
                continue
            first_tick = tick if first_tick is None else first_tick
            last_tick = tick
            pa = np.array(list(a['position'].values()))
            pb = np.array(list(b['position'].values()))
            if b['pose_valid'] and start is None:
                start = (tick, pa.copy(), pb.copy())
    if start is None or last_tick <= start[0]:
        raise ValueError('record contains no evaluation interval after stationary calibration')
    da, db = pa-start[1], pb-start[2]
    return dict(calibration_ready_after_s=(start[0]-first_tick)*.001,
                evaluated_seconds=(last_tick-start[0])*.001,
                gyro_bias_rad_s=corrected.gyro_bias.tolist(),
                baseline_delta_m=da.tolist(), corrected_delta_m=db.tolist(),
                baseline_xy_cm=float(np.linalg.norm(da[:2])*100),
                corrected_xy_cm=float(np.linalg.norm(db[:2])*100))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', help='Saved events.jsonl; robot must have been stationary to interpret displacement as drift')
    print(json.dumps(replay(parser.parse_args().events), indent=2, allow_nan=False))
