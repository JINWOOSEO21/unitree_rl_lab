import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np


TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))


class Message:
    def __init__(self, data=''):
        self.data = data


class Writer:
    instances = []

    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.messages = []
        self.initialized = False
        self.closed = False
        self.__class__.instances.append(self)

    def Init(self):
        self.initialized = True

    def Write(self, message):
        self.messages.append(json.loads(message.data))
        return True

    def Close(self):
        self.closed = True


class GyroBiasOutputTest(unittest.TestCase):
    def setUp(self):
        Writer.instances.clear()
        modules = {
            'unitree_sdk2py.core.channel': type(sys)('channel'),
            'unitree_sdk2py.idl.std_msgs.msg.dds_': type(sys)('dds_'),
        }
        modules['unitree_sdk2py.core.channel'].ChannelPublisher = Writer
        modules['unitree_sdk2py.idl.std_msgs.msg.dds_'].String_ = Message
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()

    def tearDown(self):
        self.modules.stop()

    def test_initial_calibrated_stale_and_close_messages(self):
        from go2_gyro_bias_output import GyroBiasOutput

        output = GyroBiasOutput()
        writer = Writer.instances[0]
        output.publish(False, [0.0, 0.0, 0.0], 10)
        output.publish(True, [0.001, -0.002, 0.003], 20)
        output.publish(False, [0.001, -0.002, 0.003], 30)
        output.close(30)

        self.assertEqual(writer.topic, 'rt/parkour/gyro_bias')
        self.assertTrue(writer.initialized)
        self.assertTrue(writer.closed)
        self.assertEqual([m['sequence'] for m in writer.messages], [1, 2, 3, 4, 5])
        self.assertEqual([m['calibrated'] for m in writer.messages],
                         [False, False, True, False, False])
        self.assertEqual([m['source_tick'] for m in writer.messages], [0, 10, 20, 30, 30])
        sessions = {m['session'] for m in writer.messages}
        self.assertEqual(len(sessions), 1)
        self.assertTrue(next(iter(sessions)))
        expected_keys = {'version', 'session', 'sequence', 'calibrated',
                         'bias_rad_s', 'source_tick', 'frame_id', 'units'}
        for message in writer.messages:
            self.assertEqual(set(message), expected_keys)
            self.assertEqual(message['version'], 1)
            self.assertEqual(message['frame_id'], 'base_link')
            self.assertEqual(message['units'], 'rad/s')
            self.assertEqual(len(message['bias_rad_s']), 3)

    def test_rejects_nonfinite_bias_and_regressing_tick(self):
        from go2_gyro_bias_output import GyroBiasOutput

        output = GyroBiasOutput()
        output.publish(False, [0, 0, 0], 3)
        with self.assertRaisesRegex(ValueError, 'finite'):
            output.publish(True, [0, float('nan'), 0], 4)
        with self.assertRaisesRegex(ValueError, 'regressed'):
            output.publish(False, [0, 0, 0], 2)
        output.close()

    def test_bridge_status_requires_calibration_and_fresh_advancing_lowstate(self):
        from go2_sensor_bridge import gyro_bias_status

        leg = type('Leg', (), {})()
        leg.calibrated = False
        leg.gyro_bias = np.array([0.001, -0.002, 0.003])
        calibrated, bias, tick = gyro_bias_status(leg, 900_000_000, 10,
                                                  1_000_000_000)
        self.assertFalse(calibrated)
        np.testing.assert_array_equal(bias, leg.gyro_bias)
        self.assertEqual(tick, 10)

        leg.calibrated = True
        self.assertTrue(gyro_bias_status(leg, 900_000_001, 11,
                                         1_000_000_000)[0])
        self.assertFalse(gyro_bias_status(leg, 899_999_999, 11,
                                          1_000_000_000)[0])
        self.assertFalse(gyro_bias_status(leg, None, None,
                                          1_000_000_000)[0])


if __name__ == '__main__':
    unittest.main()
