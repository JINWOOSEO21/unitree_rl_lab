"""Hardware coordinate direction and message-boundary regression checks."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from em_sidecar.go2_cloud import raw_cloud_to_base


def cloud(points, frame='utlidar_lidar'):
    data = np.asarray(points, dtype='<f4').reshape(-1, 3).tobytes()
    return NS(header=NS(frame_id=frame, stamp=NS(sec=123, nanosec=456)),
              width=len(data)//12, height=1, point_step=12, row_step=len(data),
              data=data, is_bigendian=False,
              fields=[NS(name=n, offset=4*i, datatype=7, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))])


class Go2CloudTest(unittest.TestCase):
    def test_basis_direction_translation_and_input_unchanged(self):
        msg = cloud([[0,0,0], [1,0,0], [0,1,0], [0,0,1]])
        original = msg.data
        # Independently rounded expected base positions, in metres.
        expected = [[.282160014, 0, 0],
                    [.808273915, -.838668173, .140854030],
                    [-.527975802, -.544642722, -.216896898],
                    [.540779661, .000001579, -.965979233]]
        np.testing.assert_allclose(raw_cloud_to_base(msg), expected, atol=5e-8, rtol=0)
        self.assertEqual(msg.data, original)
        self.assertEqual(msg.header.frame_id, 'utlidar_lidar')
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (123,456))

    def test_rejects_double_transform_and_unknown_frame(self):
        for frame in ('base_link', 'odom', 'lidar', ''):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                raw_cloud_to_base(cloud([[1,2,3]], frame))

    def test_finite_filter_and_empty_cloud(self):
        result = raw_cloud_to_base(cloud([[1,2,3], [np.nan,0,0], [0,np.inf,0]]))
        self.assertEqual(result.shape, (1,3))
        self.assertTrue(np.isfinite(result).all())
        self.assertEqual(raw_cloud_to_base(cloud([])).shape, (0,3))


if __name__ == '__main__':
    unittest.main()
