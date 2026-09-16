"""Frame-contract tests for the offline cloud_base policy-scan replay."""
from pathlib import Path
import sys
import unittest

import numpy as np

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

from replay_go2_base_scan import (
    backend_input_from_base_cloud,
    build_schedule,
    odom_pose,
    validate_policy_scan,
)


class CloudBaseReplayContractTest(unittest.TestCase):
    def test_fixed_10hz_schedule_is_exact_and_never_uses_future_cloud(self):
        clouds = np.array([
            23_000_000,
            81_000_000,
            147_000_000,
            221_000_000,
            282_000_000,
            351_000_000,
        ], dtype=np.int64)
        ticks, indices, ages_ms, reasons = build_schedule(clouds, "fixed10", 200.0)
        np.testing.assert_array_equal(ticks, [23_000_000, 123_000_000, 223_000_000, 323_000_000])
        np.testing.assert_array_equal(np.diff(ticks), np.full(3, 100_000_000))
        np.testing.assert_array_equal(indices, [0, 1, 3, 4])
        self.assertTrue(np.all(clouds[indices] <= ticks))
        np.testing.assert_allclose(ages_ms, [0.0, 42.0, 2.0, 41.0])
        self.assertTrue(np.all(reasons == ""))

    def test_schedule_skips_reuse_and_rejects_stale_cloud(self):
        clouds = np.array([0, 450_000_000], dtype=np.int64)
        ticks, indices, ages_ms, reasons = build_schedule(clouds, "fixed10", 200.0)
        self.assertEqual(reasons[1], "no_new_cloud")
        self.assertEqual(reasons[2], "no_new_cloud")
        self.assertEqual(reasons[3], "cloud_too_old")
        self.assertEqual(reasons[4], "cloud_too_old")
        self.assertEqual(indices[-1], 0)
        self.assertEqual(reasons[-1], "cloud_too_old")
        self.assertEqual(ages_ms[-1], 400.0)
        self.assertNotIn(1, indices)  # the cloud at 450 ms is still in the future

    def test_backend_adapter_does_not_double_transform_base_cloud(self):
        angle = np.deg2rad(37.0)
        rotation = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        position = np.array([1.2, -0.4, 0.31])
        origin = np.array([0.282160014, 0.0, 0.0])
        points_base = np.array([[0.5, 0.2, -0.3], [-0.1, 0.7, 0.4]])

        points_input, sensor_rotation, sensor_position = backend_input_from_base_cloud(
            points_base, position, rotation, origin
        )
        reconstructed = points_input @ sensor_rotation.T + sensor_position
        expected = points_base @ rotation.T + position

        np.testing.assert_allclose(reconstructed, expected, atol=1e-12)
        np.testing.assert_allclose(sensor_position, position + rotation @ origin, atol=1e-12)

    def test_robot_odom_orientation_is_used_in_wxyz_order(self):
        row = {
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
        }
        position, quat = odom_pose(row)
        np.testing.assert_array_equal(position, [1.0, 2.0, 3.0])
        expected = np.array([0.9, 0.1, 0.2, 0.3])
        expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(quat, expected)

    def test_policy_scan_requires_exact_normalized_132_shape(self):
        value = np.linspace(-1.0, 1.0, 132, dtype=np.float32)
        np.testing.assert_array_equal(validate_policy_scan(value), value)
        with self.assertRaises(ValueError):
            validate_policy_scan(np.zeros(131, dtype=np.float32))
        bad = value.copy()
        bad[17] = 1.01
        with self.assertRaises(ValueError):
            validate_policy_scan(bad)


if __name__ == "__main__":
    unittest.main()
