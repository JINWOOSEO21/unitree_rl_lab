"""Lossless sensor wire format. No ROS/DDS imports or robot operations."""
import base64
import math
import struct

MAX_LINE = 32 * 1024 * 1024


def stamp_ns(header):
    sec, ns = int(header.stamp.sec), int(header.stamp.nanosec)
    if sec < 0 or not 0 <= ns < 1_000_000_000:
        raise ValueError('invalid sensor timestamp')
    return sec*1_000_000_000 + ns


def vector(value, size):
    if len(value) != size or not all(math.isfinite(float(x)) for x in value):
        raise ValueError('invalid finite vector')
    return [float(x) for x in value]


def validate(event):
    kind = event['kind']
    if kind not in ('cloud', 'imu'):
        raise ValueError('unsupported sensor kind')
    if type(event['stamp_ns']) is not int or event['stamp_ns'] < 0:
        raise ValueError('source timestamp must be integer nanoseconds')
    expected = 'utlidar_lidar' if kind == 'cloud' else 'utlidar_imu'
    if event['frame_id'] != expected:
        raise ValueError('unexpected input frame: '+event['frame_id'])
    if kind == 'imu':
        vector(event['angular_velocity'], 3)
        vector(event['linear_acceleration'], 3)
        vector(event['orientation'], 4)  # Preserved, not consumed as attitude by LIO.
        for key in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
            if key in event: vector(event[key], 9)
        return None
    w, h, step, stride = [event[k] for k in ('width','height','point_step','row_step')]
    if any(type(x) is not int for x in (w,h,step,stride)) or min(w,h,step) <= 0 or stride < w*step:
        raise ValueError('invalid cloud layout')
    data = base64.b64decode(event['data_b64'], validate=True)
    if len(data) != stride*h or len(data) > MAX_LINE//2:
        raise ValueError('invalid cloud data length')
    fields = {f['name']:f for f in event['fields']}
    if len(fields) != len(event['fields']): raise ValueError('duplicate cloud field')
    for name in ('x','y','z','intensity','ring','time'):
        f = fields[name]; size = 2 if name == 'ring' else 4
        if f['datatype'] != (4 if name == 'ring' else 7) or f['count'] != 1 or not 0 <= f['offset'] <= step-size:
            raise ValueError('invalid field '+name)
    offset = fields['time']['offset']; fmt = '>f' if event['is_bigendian'] else '<f'
    times = [struct.unpack_from(fmt,data,y*stride+x*step+offset)[0] for y in range(h) for x in range(w)]
    if not all(math.isfinite(t) and 0 <= t <= .2 for t in times):
        raise ValueError('invalid point-relative time (expected seconds, <=0.2)')
    return max(times)


class InputClock:
    def __init__(self): self.last = {}; self.duplicates = 0; self.gaps = []
    def accept(self, event):
        validate(event)
        kind, stamp = event['kind'], event['stamp_ns']
        old = self.last.get(kind)
        if old is not None:
            if stamp < old: raise ValueError('sensor clock regressed: '+kind)
            if stamp == old:
                self.duplicates += 1
                return False
            if stamp-old > (20_000_000 if kind == 'imu' else 150_000_000):
                self.gaps.append({'kind':kind,'stamp_ns':stamp,'gap_ns':stamp-old})
        self.last[kind] = stamp
        return True
