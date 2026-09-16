"""DDS terrain output compatible with the C++ Parkour ScandotsSubscriber.

Creates only a HeightMap writer, after the caller initializes the DDS factory.
An empty data vector explicitly invalidates a previously published terrain scan.
"""
import threading

import numpy as np


class ScandotsOutput:
    def __init__(self, topic='rt/parkour/scandots'):
        # Restrict this diagnostic adapter to the terrain topic family.
        if not topic.startswith('rt/parkour/'):
            raise ValueError('scandots topic must start with rt/parkour/')
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_
        self.message_type = HeightMap_
        self.writer = ChannelPublisher(topic, HeightMap_)
        self.writer.Init()
        self.lock = threading.Lock()
        self.active = False
        self.last_source = None

    def publish(self, scan, position, source_ns, now_ns):
        values = np.asarray(scan, dtype=np.float32)
        position = np.asarray(position, dtype=np.float64)
        if values.shape != (132,) or not np.isfinite(values).all() or np.any(np.abs(values) > 1):
            raise ValueError('invalid normalized scandots')
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('invalid scandots origin')
        if not 0 <= now_ns-source_ns <= 200_000_000:
            raise ValueError('scandots source cloud is stale or future-dated')
        with self.lock:
            if self.last_source is not None and source_ns <= self.last_source:
                raise ValueError('scandots source must advance')
            message = self.message_type(
                stamp=source_ns * 1e-9, frame_id='base_yaw', resolution=0.15,
                width=12, height=11, origin=position[:2].tolist(), data=values.tolist())
            if not self.writer.Write(message):
                raise RuntimeError('DDS scandots write failed')
            self.last_source = source_ns
            self.active = True

    def invalidate(self):
        with self.lock:
            if not self.active:
                return
            message = self.message_type(stamp=0., frame_id='base_yaw', resolution=0.15,
                                        width=0, height=0, origin=[0.,0.], data=[])
            if not self.writer.Write(message):
                raise RuntimeError('DDS scandots invalidation failed')
            self.active = False

    def close(self):
        try:
            self.invalidate()
        finally:
            self.writer.Close()
