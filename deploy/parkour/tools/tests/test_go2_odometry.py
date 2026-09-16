"""Orientation differences must respect quaternion sign and wrapped angles."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_go2_odometry import orientation_summary


class OdometryOrientationTests(unittest.TestCase):
    def test_antipodal_quaternions_are_same_pose(self):
        _, summary = orientation_summary([[1, 0, 0, 0], [-1, 0, 0, 0]])
        self.assertEqual(summary["max_rotation_from_start_deg"], 0)
        np.testing.assert_allclose(summary["rpy_peak_to_peak_deg"], 0)

    def test_yaw_wrap_is_small_change(self):
        yaw = np.deg2rad([179, -179])
        q = np.column_stack([np.cos(yaw/2), np.zeros(2), np.zeros(2), np.sin(yaw/2)])
        _, summary = orientation_summary(q)
        self.assertAlmostEqual(summary["rpy_end_minus_start_deg"][2], 2)
        self.assertAlmostEqual(summary["max_rotation_from_start_deg"], 2)

    def test_zero_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            orientation_summary([[0, 0, 0, 0]])


if __name__ == "__main__":
    unittest.main()
