"""Publish the leg-odometry gyro calibration state for ``go2_ctrl``.

The payload is a small JSON heartbeat on ``rt/parkour/gyro_bias``.  It carries
only the estimated bias and its validity; raw or corrected IMU samples remain
on the existing LowState path so the controller can use the freshest sample.
"""
from __future__ import annotations

import json
import math
import threading
import uuid


class GyroBiasOutput:
    TOPIC = 'rt/parkour/gyro_bias'

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        self.message_type = String_
        self.writer = ChannelPublisher(self.TOPIC, String_)
        self.writer.Init()
        self.lock = threading.Lock()
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.last_source_tick = 0
        self.closed = False
        # A late-starting controller will subsequently receive heartbeats, but
        # an already-running controller must also see this session invalidate
        # any bias retained from an earlier bridge process immediately.
        self.publish(False, [0.0, 0.0, 0.0], 0)

    @staticmethod
    def _bias(values):
        bias = [float(value) for value in values]
        if len(bias) != 3 or not all(math.isfinite(value) for value in bias):
            raise ValueError('gyro bias must contain three finite values')
        return bias

    def publish(self, calibrated, bias_rad_s, source_tick):
        bias = self._bias(bias_rad_s)
        source_tick = int(source_tick)
        if source_tick < 0:
            raise ValueError('gyro bias source tick must be non-negative')
        with self.lock:
            if self.closed:
                raise RuntimeError('gyro bias publisher is closed')
            if source_tick < self.last_source_tick:
                raise ValueError('gyro bias source tick regressed')
            self.sequence += 1
            packet = {
                'version': 1,
                'session': self.session,
                'sequence': self.sequence,
                'calibrated': bool(calibrated),
                'bias_rad_s': bias,
                'source_tick': source_tick,
                'frame_id': 'base_link',
                'units': 'rad/s',
            }
            message = self.message_type(
                data=json.dumps(packet, allow_nan=False, separators=(',', ':')))
            if not self.writer.Write(message):
                raise RuntimeError('DDS gyro bias write failed')
            self.last_source_tick = source_tick

    def close(self, source_tick=None):
        with self.lock:
            if self.closed:
                return
            tick = self.last_source_tick if source_tick is None else int(source_tick)
        try:
            self.publish(False, [0.0, 0.0, 0.0], tick)
        finally:
            with self.lock:
                self.closed = True
            self.writer.Close()
